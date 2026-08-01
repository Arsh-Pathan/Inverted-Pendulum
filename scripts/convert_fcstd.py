import os
import sys

def convert_fcstd_to_stl(fcstd_path="models/assembly_model.FCStd"):
    """
    Attempts to import FreeCAD's Python library or use freecadcmd to convert 
    assembly_model.FCStd shapes directly into STL mesh files.
    """
    fcstd_path = os.path.abspath(fcstd_path)
    if not os.path.exists(fcstd_path):
        print(f"[CONVERTER ERROR] File not found: {fcstd_path}")
        return False

    # Standard Windows FreeCAD installation paths
    freecad_paths = [
        r"C:\Program Files\FreeCAD 0.21\bin",
        r"C:\Program Files\FreeCAD 0.20\bin",
        r"C:\Program Files\FreeCAD 1.0\bin",
        r"C:\Program Files\FreeCAD\bin",
    ]

    for p in freecad_paths:
        if os.path.exists(p) and p not in sys.path:
            sys.path.append(p)

    try:
        import FreeCAD
        import Mesh
        print(f"[FreeCAD] Successfully loaded FreeCAD engine! Opening {fcstd_path}...")
        doc = FreeCAD.openDocument(fcstd_path)
        
        cart_stl = os.path.abspath("models/cart/cart_model.stl")
        pendulum_stl = os.path.abspath("models/pendulum/pendulum_model.stl")
        
        cart_objs = [obj for obj in doc.Objects if "Cart" in obj.Name or "Cart" in getattr(obj, "Label", "")]
        pendulum_objs = [obj for obj in doc.Objects if "Pendulum" in obj.Name or "Pendulum" in getattr(obj, "Label", "")]
        
        if cart_objs:
            Mesh.export(cart_objs, cart_stl)
            print(f"[FreeCAD] Exported Cart assembly mesh to: {cart_stl}")
        if pendulum_objs:
            Mesh.export(pendulum_objs, pendulum_stl)
            print(f"[FreeCAD] Exported Pendulum assembly mesh to: {pendulum_stl}")
            
        FreeCAD.closeDocument(doc.Name)
        return True
    except Exception as e:
        print(f"[FreeCAD CONVERTER NOTE] Could not convert dynamically: {e}")
        return False

if __name__ == "__main__":
    convert_fcstd_to_stl()
