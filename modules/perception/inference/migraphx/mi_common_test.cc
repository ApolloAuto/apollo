#include "modules/perception/inference/migraphx/mi_common.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace inference {

TEST(RTModifyPoolingParamTest, test) {
  {
    PoolingParameter pool_param;
    pool_param.set_kernel_size(1);
    pool_param.set_stride(1);
    ACHECK(modify_pool_param(&pool_param));
    EXPECT_EQ(pool_param.kernel_w(), 1);
    EXPECT_EQ(pool_param.kernel_h(), 1);
  }
  {
    PoolingParameter pool_param;
    pool_param.set_kernel_h(0);
    ACHECK(!modify_pool_param(&pool_param));
  }
  {
    PoolingParameter pool_param;
    pool_param.set_kernel_w(0);
    ACHECK(!modify_pool_param(&pool_param));
  }
  {
    PoolingParameter pool_param;
    pool_param.set_stride(1);
    pool_param.set_kernel_size(1);
    ACHECK(modify_pool_param(&pool_param));
    EXPECT_EQ(pool_param.stride_w(), 1);
    EXPECT_EQ(pool_param.stride_h(), 1);
  }
  {
    PoolingParameter pool_param;
    pool_param.set_kernel_size(1);
    pool_param.set_stride_h(0);
    ACHECK(!modify_pool_param(&pool_param));
  }
  {
    PoolingParameter pool_param;
    pool_param.set_stride_w(0);
    pool_param.set_kernel_size(1);
    ACHECK(!modify_pool_param(&pool_param));
  }
  {
    PoolingParameter pool_param;
    pool_param.set_pad(1);
    pool_param.set_kernel_size(1);
    pool_param.set_stride(1);
    ACHECK(modify_pool_param(&pool_param));
    EXPECT_EQ(pool_param.pad_h(), 1);
    EXPECT_EQ(pool_param.pad_w(), 1);
  }
  {
    PoolingParameter pool_param;
    pool_param.set_kernel_size(1);
    pool_param.set_stride(1);
    ACHECK(modify_pool_param(&pool_param));
    EXPECT_EQ(pool_param.pad_h(), 0);
    EXPECT_EQ(pool_param.pad_w(), 0);
  }
}

TEST(RTParseConvParamTest, test) {
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.add_stride(1);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(ParserConvParam(conv_param, &tensorrt_param));
    EXPECT_EQ(tensorrt_param.kernel_h, 3);
    EXPECT_EQ(tensorrt_param.kernel_w, 3);
    EXPECT_EQ(tensorrt_param.padding_h, 1);
    EXPECT_EQ(tensorrt_param.padding_w, 1);
    EXPECT_EQ(tensorrt_param.stride_h, 1);
    EXPECT_EQ(tensorrt_param.stride_w, 1);
  }
  {
    ConvolutionParameter conv_param;
    conv_param.clear_kernel_size();
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.clear_kernel_size();
    conv_param.set_kernel_h(3);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.clear_kernel_size();
    conv_param.set_kernel_w(3);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.set_pad_h(1);
    conv_param.add_stride(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(ParserConvParam(conv_param, &tensorrt_param));
    EXPECT_EQ(tensorrt_param.kernel_h, 3);
    EXPECT_EQ(tensorrt_param.kernel_w, 3);
    EXPECT_EQ(tensorrt_param.padding_h, 1);
    EXPECT_EQ(tensorrt_param.padding_w, 0);
    EXPECT_EQ(tensorrt_param.stride_h, 1);
    EXPECT_EQ(tensorrt_param.stride_w, 1);
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.set_pad_w(1);
    conv_param.add_stride(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(ParserConvParam(conv_param, &tensorrt_param));
    EXPECT_EQ(tensorrt_param.kernel_h, 3);
    EXPECT_EQ(tensorrt_param.kernel_w, 3);
    EXPECT_EQ(tensorrt_param.padding_h, 0);
    EXPECT_EQ(tensorrt_param.padding_w, 1);
    EXPECT_EQ(tensorrt_param.stride_h, 1);
    EXPECT_EQ(tensorrt_param.stride_w, 1);
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.set_pad_w(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.set_pad_h(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.add_stride(1);
    conv_param.set_stride_h(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.add_stride(1);
    conv_param.set_stride_w(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.set_stride_h(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
  {
    ConvolutionParameter conv_param;
    conv_param.add_kernel_size(3);
    conv_param.add_pad(1);
    conv_param.set_stride_w(1);
    conv_param.add_dilation(0);
    apollo::perception::inference::ConvParam tensorrt_param;
    ACHECK(!ParserConvParam(conv_param, &tensorrt_param));
  }
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
