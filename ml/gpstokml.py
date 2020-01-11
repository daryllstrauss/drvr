# <?xml version="1.0" encoding="UTF-8"?>
# <kml xmlns="http://www.opengis.net/kml/2.2">
#   <Document>
#     <name>Paths</name>
#     <description>Examples of paths. Note that the tessellate tag is by default
#       set to 0. If you want to create tessellated lines, they must be authored
#       (or edited) directly in KML.</description>
#     <Style id="yellowLineGreenPoly">
#       <LineStyle>
#         <color>7f00ffff</color>
#         <width>4</width>
#       </LineStyle>
#       <PolyStyle>
#         <color>7f00ff00</color>
#       </PolyStyle>
#     </Style>
#     <Placemark>
#       <name>Absolute Extruded</name>
#       <description>Transparent green wall with yellow outlines</description>
#       <styleUrl>#yellowLineGreenPoly</styleUrl>
#       <LineString>
#         <extrude>1</extrude>
#         <tessellate>1</tessellate>
#         <altitudeMode>absolute</altitudeMode>
#         <coordinates> -112.2550785337791,36.07954952145647,2357
#           -112.2549277039738,36.08117083492122,2357
#           -112.2552505069063,36.08260761307279,2357
#           -112.2564540158376,36.08395660588506,2357
#           -112.2580238976449,36.08511401044813,2357
#           -112.2595218489022,36.08584355239394,2357
#           -112.2608216347552,36.08612634548589,2357
#           -112.262073428656,36.08626019085147,2357
#           -112.2633204928495,36.08621519860091,2357
#           -112.2644963846444,36.08627897945274,2357
#           -112.2656969554589,36.08649599090644,2357 
#         </coordinates>
#       </LineString>
#     </Placemark>
#   </Document>
# </kml>

import os
import sys
import json

def header():
    print('<?xml version="1.0" encoding="UTF-8"?>')
    print('<kml xmlns="http://www.opengis.net/kml/2.2">')
    print('  <Document>')

def style():
    print('    <Style id="green">')
    print('      <LineStyle>')
    print('        <color>00ff0000</color>')
    print('        <width>2</width>')
    print('      </LineStyle>')
    print('    </Style>')

def path(data):
    print('    <Placemark>')
    print('      <styleUrl>#green</styleUrl>')
    print('      <LineString>')
    print('        <tessellate>1</tessellate>')
    print('        <altitudeMode>relativeToGround</altitudeMode>')
    print('        <coordinates>')
    for rec in data:
        print(f'          {rec["lon"]},{rec["lat"]},0')
    print('        </coordinates>')    
    print('      </LineString>')
    print('    </Placemark>')

def footer():
    print('  </Document>')
    print('</kml>')

def readData(dirname):
    files = os.listdir(dirname)
    files.sort()
    result = []
    for filename in files:
        if filename[:9] == "position-":
            with open(f'{dirname}/{filename}', "r") as f:
                data = json.load(f)
            result.append(data)
    return result

def main():
    dirname = sys.argv[-1]
    data = readData(dirname)
    header()
    style()
    path(data)
    footer()

if __name__ == '__main__':
    main()

