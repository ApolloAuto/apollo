#ifndef ROTATIONMATRIX_H
#define ROTATIONMATRIX_H

static const int kNumReal3DToImage = 12;
static const int kNumImageToReal3D = 9;

struct RotationMatrix
{
    //[tempX]     [ m[0],     m[1],       m[2],   m[3] ]      [real3DX]
    //[tempY]  =  [ m[4],     m[5],       m[6],   m[7] ]  *   [real3Dy]
    //[  Z  ]     [ m[8],     m[9],       m[10],  m[11]]      [real3DZ]
    //                                                        [   1   ]
    // final:
    //[imageX]      [tempX]
    //[imageY]  =   [tempY] / Z
    //[   1  ]      [   Z  ]
    float real3DToImage[kNumReal3DToImage];

    //[tempX]     [ m[0],     m[1],       m[2] ]      [imageX]
    //[tempY]  =  [ m[3],     m[4],       m[5] ]  *   [imageY]
    //[tempZ]     [ m[6],     m[7],       m[8] ]      [   1  ]
    // in: (assume: one of real3DX, real3DY, real3DZ is available -> get k -> get all of real3DX, real3DY, real3DZ）
    // real3DX / tempX = real3DY / tempY = real3DZ / tempZ = k;
    // final: (assume: real3DX', real3DY', real3DZ' are 3D information with car center bias as origin, Translation Matrix is bias matrix)
    //[real3DX']      [real3DX]
    //[real3DY']  =   [real3DY] - Translation Matrix
    //[real3DZ']      [real3DZ]
    float imageToReal3D[kNumImageToReal3D];
};

#endif // ROTATIONMATRIX_H
