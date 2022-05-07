# A tool to transform coodinate between wgs84 and UTM 

1. start the binary
./bazel-bin/modules/tools/wgs84_utm_transform/coord_transfrom
2. input format is like this:`0|1 zone_id x y.   0 is 84 to UTM | 1 is UTM to 84`
3. the first flag 0 or 1, 0 stands for transform from wgs84 to UTM, 1 stands for transform coordinate from UTM to wgs84
4. the second flag zone_id means UTM zone_id, you can look up it by you latitude. https://mangomap.com/robertyoung/maps/69585/what-utm-zone-am-i-in-#
5. x,y is you longitude and latitude (in wgs 84),or x,y (in UTM)
6. then program will return your transformed coordinate