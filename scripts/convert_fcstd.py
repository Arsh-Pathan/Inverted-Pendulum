import os
import sys
import subprocess

def convert_fcstd_to_stl(fcstd_path="models/assembly_model.FCStd"):
    """
    Converts assembly_model.FCStd shapes (Cart, Pendulum, Linear Rail / Track)
    directly into unified STL mesh files using FreeCAD 1.1's native engine.
    """
    fcstd_path = os.path.abspath(fcstd_path)
    if not os.path.exists(fcstd_path):
        print(f"[CONVERTER ERROR] File not found: {fcstd_path}")
        return False

    freecad_exe = r"C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe"
    cart_stl = os.path.abspath("models/cart/cart_model.stl")
    pendulum_stl = os.path.abspath("models/pendulum/pendulum_model.stl")
    track_stl = os.path.abspath("models/track_model.stl")

    if os.path.exists(freecad_exe):
        py_script = f"""
import FreeCAD, Mesh
doc = FreeCAD.openDocument(r'{fcstd_path}')
cart_objs = [o for o in doc.Objects if 'Cart' in o.Name or 'Cart' in getattr(o, 'Label', '')]
pendulum_objs = [o for o in doc.Objects if 'Pendulum' in o.Name or 'Pendulum' in getattr(o, 'Label', '')]
track_objs = [o for o in doc.Objects if 'Rail' in o.Name or 'Rail' in getattr(o, 'Label', '') or 'Track' in o.Name or 'Linear' in o.Name]

if cart_objs:
    Mesh.export(cart_objs, r'{cart_stl}')
    print('Exported Cart STL')
if pendulum_objs:
    Mesh.export(pendulum_objs, r'{pendulum_stl}')
    print('Exported Pendulum STL')
if track_objs:
    Mesh.export(track_objs, r'{track_stl}')
    print('Exported Track STL')
"""
        script_file = os.path.abspath("scratch/convert_script.py")
        os.makedirs("scratch", exist_ok=True)
        with open(script_file, "w", encoding="utf-8") as f:
            f.write(py_script)

        try:
            print(f"[FreeCAD 1.1] Extracting Cart, Pendulum, and Track from {fcstd_path}...")
            res = subprocess.run([freecad_exe, script_file], capture_output=True, text=True, timeout=15)
            print(f"[FreeCAD Output] {res.stdout.strip()}")
            return True
        except Exception as e:
            print(f"[FreeCAD Exec Note] {e}")

    return False

if __name__ == "__main__":
    convert_fcstd_to_stl()
