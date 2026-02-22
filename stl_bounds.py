import struct
def get_stl_bounds(filename):
    with open(filename, 'rb') as f:
        f.read(80) # header
        num_triangles = struct.unpack('<I', f.read(4))[0]
        min_vals = [float('inf')]*3
        max_vals = [float('-inf')]*3
        for _ in range(num_triangles):
            normal = struct.unpack('<3f', f.read(12))
            for _ in range(3):
                v = struct.unpack('<3f', f.read(12))
                for i in range(3):
                    if v[i] < min_vals[i]: min_vals[i] = v[i]
                    if v[i] > max_vals[i]: max_vals[i] = v[i]
            f.read(2) # attributes
    return [round(x, 4) for x in min_vals], [round(x, 4) for x in max_vals]

print("base_link bounds:", get_stl_bounds('src/amr_mtt/amr_mtt_bot/meshes/amr_mtt_model/base_link.STL'))
