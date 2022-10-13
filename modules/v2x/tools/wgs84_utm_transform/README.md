# A tool to transform coodinate between wgs84 and UTM

1. start the binary
./bazel-bin/modules/v2x/tools/wgs84_utm_transform/coord_transfrom
2. input format is like this:`0|1 zone_id x y.   0 is WGS84 to UTM | 1 is UTM to WGS84`
3. the first flag 0 or 1, 0 stands for transform from WGS84 to UTM, 1 stands for transform coordinate from UTM to WGS84
4. the second flag zone_id means UTM zone_id, you can look up it by you latitude. https://mangomap.com/robertyoung/maps/69585/what-utm-zone-am-i-in-#
5. x,y is you longitude and latitude (in WGS 84),or x,y (in UTM)
6. then program will return your transformed coordinate
7. example:

    a) WGS84 to UTM, zone_id is 50, lon is 116.660407, lat is 39.741546.
    ```
    ./bazel-bin/modules/v2x/tools/wgs84_utm_transform/coord_transfrom

    0 50 116.660407 39.741546

    result: 470903.420894 4399127.122057
    ```
    b) UTM to WGS84, zone_id is 50, x is 470903.420894, y is 4399127.122057.
    ```
    ./bazel-bin/modules/v2x/tools/wgs84_utm_transform/coord_transfrom

    1 50 470903.420894 4399127.122057

    result: 116.660407 39.741546
    ```
