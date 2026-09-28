import json

with open("data/maps/latest.json", "r", encoding="utf-8") as f:
    doc = json.load(f)

exp = doc.get("exploration", {})
grid = exp.get("cell_grid", {})
print("Anchor pose:", doc.get("anchor_pose"))
print("Pose:", doc.get("pose"))
cells = {tuple(c["index"]): c["sides"] for c in grid.get("cells", [])}

for cell in sorted(cells.keys()):
    if -1 <= cell[0] <= 2 and -1 <= cell[1] <= 2:
        print(f"Cell {cell}:")
        for side, data in cells[cell].items():
            print(f"  {side}: state={data.get('state')} measured_from={data.get('measured_from')} range={data.get('range_mm')}")
