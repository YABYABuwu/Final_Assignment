import json

with open("data/targets.json", "r", encoding="utf-8") as f:
    doc = json.load(f)

for t in doc.get("targets", []):
    pos = tuple(t.get("pos", []))
    color = t.get("color")
    shape = t.get("shape")

    # 1. Red circle at (0, 0) -> LEFT wall (y+)
    if pos == (0, 0) and color == "red":
        t["side"] = "y+"

    # 2. Yellow circle at (1, 1) -> RIGHT wall (y-)
    elif pos == (1, 1) and color == "yellow" and shape == "circle":
        t["side"] = "y-"

    # 3. Green circle & Yellow square at (1, 2) -> LEFT wall (y+)
    elif pos == (1, 2) and (color == "green" or color == "yellow"):
        t["side"] = "y+"

    # 4. Green square/vertical at (1, 1) -> FRONT wall (x+)
    elif pos == (1, 1) and color == "green":
        t["side"] = "x+"

    # 5. Yellow square/vertical at (0, 1) -> BACK wall (x-)
    elif pos == (0, 1) and color == "yellow":
        t["side"] = "x-"

with open("data/targets.json", "w", encoding="utf-8") as f:
    json.dump(doc, f, indent=2)

print("Successfully updated targets.json with correct wall sides!")
