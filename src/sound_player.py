"""Audio playback and startup sound manager for DJI RoboMaster EP."""

import logging
from pathlib import Path
import subprocess

logger = logging.getLogger(__name__)

# SFX directory default (Final_Assignment/sfx)
# SFX directory default (Final_Assignment/sfx)
SFX_DIR = Path(__file__).resolve().parent.parent / "sfx"

# ระดับความดังเริ่มต้น (ปรับเพิ่ม/ลดได้ที่นี่):
# 0.08 = เบาที่สุด, 0.16 = ดังขึ้น 1 ระดับ (ค่าปัจจุบัน), 0.30 = ปานกลาง, 0.50+ = ดัง
DEFAULT_VOLUME = 0.70


def _get_ffmpeg_exe():
    """Find ffmpeg executable from imageio_ffmpeg, PATH, or local environment."""
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        pass
    return "ffmpeg"


def convert_audio(
    input_path: Path,
    output_path: Path,
    volume_factor: float = DEFAULT_VOLUME,
    sample_rate: int = 48000,
    channels: int = 1,
) -> bool:
    """
    Convert an audio file (e.g. .mp3, .wav) to the RoboMaster requirement:
    - 48 kHz sampling rate
    - Mono (1 channel)
    - 16-bit PCM WAV
    - Custom volume factor (e.g. 0.16)
    """
    input_path = Path(input_path)
    output_path = Path(output_path)

    if not input_path.exists():
        logger.error(f"Input audio file not found: {input_path}")
        return False

    output_path.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg_exe = _get_ffmpeg_exe()

    # ffmpeg volume filter: volume=0.08 (-22 dB, very soft)
    cmd = [
        ffmpeg_exe,
        "-y",
        "-i", str(input_path),
        "-ac", str(channels),
        "-ar", str(sample_rate),
        "-filter:a", f"volume={volume_factor:.4f}",
        str(output_path),
    ]

    try:
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        logger.info(f"Successfully converted {input_path.name} -> {output_path.name} (volume={volume_factor})")
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"FFmpeg conversion failed: {e.stderr}")
        return False
    except Exception as e:
        logger.error(f"Audio conversion failed: {e}")
        return False


def get_sound_path(sound_name: str, volume_factor: float = DEFAULT_VOLUME) -> Path:
    """
    Resolve sound file path in sfx/.
    If input is .mp3, auto-generate a quiet .wav file.
    """
    file_path = Path(sound_name)
    if not file_path.is_absolute():
        file_path = SFX_DIR / sound_name

    # If the file already exists as a .wav
    if file_path.suffix.lower() == ".wav" and file_path.exists():
        return file_path

    # If asking for .mp3 or wav does not exist, look for candidate
    base_stem = file_path.stem
    wav_target = SFX_DIR / f"{base_stem}_vol_{int(volume_factor * 100)}.wav"
    if wav_target.exists():
        return wav_target

    # Look for matching source (.mp3 or .wav)
    src_candidates = [
        SFX_DIR / f"{base_stem}.mp3",
        SFX_DIR / f"{base_stem}.wav",
        file_path.with_suffix(".mp3"),
        file_path,
    ]
    for src in src_candidates:
        if src.exists():
            convert_audio(src, wav_target, volume_factor=volume_factor)
            if wav_target.exists():
                return wav_target

    # Fallback to startup.wav if exists
    default_startup = SFX_DIR / "startup.wav"
    if default_startup.exists():
        return default_startup

    return file_path


def play_startup_sound(
    ep_robot,
    sound_file: str = "startup.wav",
    volume_factor: float = DEFAULT_VOLUME,
    wait: bool = True,
):
    """
    Play the startup sound on RoboMaster EP.
    
    :param ep_robot: Connected robomaster.robot.Robot instance
    :param sound_file: Filename in Final_Assignment/sfx/ (e.g. 'startup.wav', 'fah.mp3')
    :param volume_factor: Audio gain multiplier (default: DEFAULT_VOLUME = 0.16)
    :param wait: If True, blocks until the audio finishes playing
    :return: Action object or None
    """
    sound_path = get_sound_path(sound_file, volume_factor=volume_factor)
    if not sound_path.exists():
        print(f"[SoundPlayer] Sound file not found: {sound_path}")
        return None

    print(f"[SoundPlayer] Playing sound: {sound_path.name} on robot speaker...")
    try:
        action = ep_robot.play_audio(str(sound_path))
        if action and wait:
            action.wait_for_completed()
        return action
    except Exception as e:
        print(f"[SoundPlayer] Failed to play audio: {e}")
        return None


class SoundPlayer:
    """Helper class to manage sounds for RoboMaster."""

    def __init__(self, ep_robot, sfx_dir: Path = SFX_DIR, default_volume: float = DEFAULT_VOLUME):
        self.robot = ep_robot
        self.sfx_dir = Path(sfx_dir)
        self.default_volume = default_volume

    def play_startup(self, sound_name: str = "startup.wav", wait: bool = True):
        """Play startup sound with the default volume."""
        return play_startup_sound(
            self.robot,
            sound_file=sound_name,
            volume_factor=self.default_volume,
            wait=wait,
        )

    def play(self, sound_name: str, volume: float = None, wait: bool = True):
        """Play any sound in sfx/ directory."""
        vol = volume if volume is not None else self.default_volume
        return play_startup_sound(self.robot, sound_file=sound_name, volume_factor=vol, wait=wait)


if __name__ == "__main__":
    # Re-generate startup.wav with the new DEFAULT_VOLUME (0.16)
    mp3_source = SFX_DIR / "fah.mp3"
    wav_output = SFX_DIR / "startup.wav"
    quiet_output = SFX_DIR / "fah_quiet.wav"

    print(f"Converting fah.mp3 with DEFAULT_VOLUME ({DEFAULT_VOLUME})...")
    convert_audio(mp3_source, quiet_output, volume_factor=DEFAULT_VOLUME)
    convert_audio(mp3_source, wav_output, volume_factor=DEFAULT_VOLUME)
    print(f"Done! Files ready at:\n  - {quiet_output}\n  - {wav_output}")
