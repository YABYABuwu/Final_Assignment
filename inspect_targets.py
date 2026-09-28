import json

with open("data/targets.json", "r", encoding="utf-8") as f:
    doc = json.load(f)

for t in doc.get("targets", []):
    obs = t.get("observation", {})
    pose = obs.get("pose", [0, 0, 0])
    print(f"{t['id']:4s} pos={str(t['pos']):7s} side={str(t.get('side')):4s} color={t['color']:7s} shape={t['shape']:10s} pose_yaw={pose[2]:6.2f} gyaw={obs.get('gimbal_yaw_deg', 0.0):6.2f}")
