#pragma once

#include <vector>

namespace nvinfer1 {

#define TRT_DEPRECATED

//!
//! \enum DimensionType
//! \brief The type of data encoded across this dimension.
//!
enum class DimensionType : int32_t
{
    kSPATIAL = 0, //!< Elements correspond to different spatial data.
    kCHANNEL = 1, //!< Elements correspond to different channels.
    kINDEX = 2,   //!< Elements correspond to different batch index.
    kSEQUENCE = 3 //!< Elements correspond to different sequence values.
};


//!
//! \class Dims
//! \brief Structure to define the dimensions of a tensor.
//!
//! \note: Currently the following formats are supported for layer inputs and outputs:
//! * zero or more index dimensions followed by one channel and two spatial dimensions (e.g. CHW)
//! * one time series dimension followed by one index dimension followed by one channel dimension (i.e. TNC)
//!
//! TensorRT can also return an invalid dims structure. This structure is represented by nbDims == -1
//! and d[i] == 0 for all d.
//!
//! TensorRT can also return an "unknown rank" dims structure. This structure is represented by nbDims == -1
//! and d[i] == -1 for all d.
//!
class Dims
{
public:
    static const int32_t MAX_DIMS = 8;           //!< The maximum number of dimensions supported for a tensor.
    int32_t nbDims;                              //!< The number of dimensions.
    int32_t d[MAX_DIMS];                         //!< The extent of each dimension.
    TRT_DEPRECATED DimensionType type[MAX_DIMS]; //!< The type of each dimension, provided for backwards compatibility
                                                 //!< and will be removed in TensorRT 8.0.
};

//!
//! \class Dims4
//! \brief Descriptor for four-dimensional data.
//!
class Dims4 : public Dims
{
public:
    //!
    //! \brief Construct an empty Dims2 object.
    //!
    Dims4()
    {
        nbDims = 4;
        d[0] = d[1] = d[2] = d[3] = 0;
    }

    //!
    //! \brief Construct a Dims4 from 4 elements.
    //!
    //! \param d0 The first element.
    //! \param d1 The second element.
    //! \param d2 The third element.
    //! \param d3 The fourth element.
    //!
    Dims4(int32_t d0, int32_t d1, int32_t d2, int32_t d3)
    {
        nbDims = 4;
        d[0] = d0;
        d[1] = d1;
        d[2] = d2;
        d[3] = d3;
    }
};

class IPlugin {
 public:
  virtual int32_t getNbOutputs() const = 0;

  virtual Dims getOutputDimensions(int32_t index, const Dims* inputs,
                                   int32_t nbInputDims) = 0;

  virtual void configure(const Dims* inputDims, int32_t nbInputs,
                         const Dims* outputDims, int32_t nbOutputs,
                         int32_t maxBatchSize) = 0;

  virtual int32_t initialize() = 0;

  virtual void terminate() = 0;

  virtual size_t getWorkspaceSize(int32_t maxBatchSize) const = 0;

  virtual int32_t enqueue(int32_t batchSize, const void* const* inputs,
                          void** outputs, void* workspace,
                          cudaStream_t stream) = 0;

  virtual size_t getSerializationSize() = 0;

  virtual void serialize(void* buffer) = 0;

  virtual ~IPlugin() {}
};

}  // namespace nvinfer1