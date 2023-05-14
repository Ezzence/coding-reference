import numpy as np
import shapely as sh
from shapely import LineString, Polygon
import requests
#import json


"""
DISCLAIMER: This is Proof-of-Concept code for fitting rectangles (blocks) inside an arbitrary input polygon
It is not a perfect solution, and inteded for visualization only
The algorithm is the following:
1. Form an oriented bounding box around the input polygon (using unideal library function)
2. Get the longest edge of the bounding box and it's normal
3. Place "blocks" oriented parallel to this longest line, repeat until reaching the other edge of the bounding box
4. Exclude blocks that are not completely overlapped by the input polygon (using library functions)
5. Return remaining valid blocks

Suggestions for improvement:
- Conversion between gps coordinates and local ENU coordinates https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
- Placing blocks with perpendicular orientation to the original ones in second iteration and then removing overlaps
    - This would improve filling rate around some edges
- Using the (extended) longest edge of the input polygon, instead of the longest edge of the bounding box
    - This would improve filling rate, but would require more manual coding with numpy
"""

BLOCK_SPACING = 2*0.000009 # meters times inaccurate latlong approx of 1 meter
BLOCK_LENGTH = 30*0.000009
BLOCK_WIDTH = 12*0.000009
ROW_SPACING = 0.42*0.000009
ROW_WIDTH = 0.76*0.000009
URL_JSON_IN = 'https://reqbin.com/echo/get/json'
URL_JSON_OUT = 'https://reqbin.com/echo/post/json'

data_in = []
data_in_holes = [[]]
polygon = Polygon()
data_out = []
data_out_rows = []


def get_longest_line(poly):
    dist = 0
    dist_max = 0
    seg = [(0,0), (0,0)]
    seg_max = [(0,0), (0,0)]
    perp = []
    perp_max = []
    for idx, line in enumerate(poly):
        if idx >= len(poly) - 2:
            seg = [line, poly[0]]
            dist = 0 # missing next line
        else:
            seg = [line, poly[idx+1]]
            dist = sh.length(LineString(seg))
            perp = [poly[idx+1], poly[idx+2]]
        
        if dist > dist_max:
            dist_max = dist
            seg_max = seg
            perp_max = perp
    return seg_max, perp_max



if __name__ == "__main__":
    
    header_json = {'Accept': 'application/json'}
    r = requests.get(URL_JSON_IN, headers=header_json) # INPUT JSON HERE
    r.raise_for_status()  # raises exception when not a 2xx response
    if r.status_code == 204:
        print("Error: Empty response")
    else:
        dict_in = r.json()

    """
    f = open("dat.json")
    r = json.load(f)
    """

    # Input of necessary polygon
    data_in = dict_in["field"]
    # Optional extra parameters
    if "forbidden" in dict_in:
        data_in_holes = dict_in["forbidden"]
    if "block_length" in dict_in:
        BLOCK_LENGTH = dict_in["block_length"]
    if "block_width" in dict_in:
        BLOCK_WIDTH = dict_in["block_width"]

    if type(data_in) == list and len(data_in) > 2 and type(data_in_holes[0]) == list:
        
        data_in.append(data_in[0]) # append first point last to create LinearRing
        
        hole_polygons = []
        for hole in data_in_holes:
            if hole:
                hole.append(hole[0])
                hole_polygons.append(Polygon(hole))

        polygon = Polygon(data_in, data_in_holes)
        bounds = sh.oriented_envelope(polygon)
        #print(bounds)
        ring = bounds.exterior.coords
        segment, perp = get_longest_line(ring)

        l1 = np.array(segment[0])
        l2 = np.array(segment[1])
        p = np.array(perp[1])

        l_size = np.linalg.norm(l2 - l1)
        l_d = (l2 - l1)/l_size

        p_size = np.linalg.norm(p - l2)
        p_d = (p - l2)/p_size


        b1 = l1 + BLOCK_SPACING*p_d + BLOCK_SPACING*l_d
        b2 = b1 + BLOCK_LENGTH*l_d
        b3 = b2 + BLOCK_WIDTH*p_d
        b4 = b3 - BLOCK_LENGTH*l_d

        sh.prepare(polygon)

        for id1 in range(np.ceil(p_size/(BLOCK_SPACING + BLOCK_WIDTH)).astype(int)):
            for id2 in range(np.ceil(l_size/(BLOCK_SPACING + BLOCK_LENGTH)).astype(int)):
                if sh.contains(polygon, LineString([b1, b2, b3, b4])):
                    data_out.append([tuple(b1), tuple(b2), tuple(b3), tuple(b4)])

                    c1 = b1
                    c2 = b2
                    c3 = b2 + ROW_WIDTH*p_d
                    c4 = c3 - BLOCK_LENGTH*l_d

                    for id3 in range(np.floor(BLOCK_WIDTH/(ROW_SPACING + ROW_WIDTH)).astype(int)):
                        data_out_rows.append([tuple(c1), tuple(c2), tuple(c3), tuple(c4)])

                        c1 = c4 + ROW_SPACING*p_d
                        c2 = c1 + BLOCK_LENGTH*l_d
                        c3 = c2 + ROW_WIDTH*p_d
                        c4 = c3 - BLOCK_LENGTH*l_d

                b1 = b2 + BLOCK_SPACING*l_d
                b2 = b1 + BLOCK_LENGTH*l_d
                b3 = b2 + BLOCK_WIDTH*p_d
                b4 = b3 - BLOCK_LENGTH*l_d

            b1 = l1 + ((id1+1)*(BLOCK_SPACING + BLOCK_WIDTH))*p_d + BLOCK_SPACING*l_d
            b2 = b1 + BLOCK_LENGTH*l_d
            b3 = b2 + BLOCK_WIDTH*p_d
            b4 = b3 - BLOCK_LENGTH*l_d

        dict_out = {"blocks": data_out, "rows": data_out_rows}
        requests.post(URL_JSON_OUT, json=dict_out) # OUTPUT JSON HERE

        """
        json_out = json.dumps(dict_out, indent=4)
        with open("out.json", "w") as outfile:
            outfile.write(json_out)
        """
        #print(data_out)
        print("Finished: Block generation")


    else:
        #print(data_in)
        print("Error: Missing or badly formatted data")
