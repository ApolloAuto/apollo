# Introduction
The camera overlap filter module used to fuse detection results from multiple cameras.

1. overlap area
We split car area as 9 blocks, the overlap area is left front, left rear,
right front, right rear.

After indexing, judge whether it is the same car according to the overlap rate.
