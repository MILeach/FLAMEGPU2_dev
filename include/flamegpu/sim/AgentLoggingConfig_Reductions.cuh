#ifndef INCLUDE_FLAMEGPU_SIM_AGENTLOGGINGCONFIG_REDUCTIONS_CUH_
#define INCLUDE_FLAMEGPU_SIM_AGENTLOGGINGCONFIG_REDUCTIONS_CUH_

#include <functional>

namespace flamegpu_internal {
// standard_deviation_add_impl is a manual expansion of FLAMEGPU_CUSTOM_REDUCTION()
// This is required, so that the instance definition can be extern'd, as it is placed in a header
struct standard_deviation_add_impl {
 public:
    template <typename OutT>
    struct binary_function : public std::binary_function<OutT, OutT, OutT> {
        __device__ __forceinline__ OutT operator()(const OutT &a, const OutT &b) const;
    };
};
// standard_deviation_subtract_mean_impl is a manual expansion of FLAMEGPU_CUSTOM_TRANSFORM()
// This is required, so that the instance definition can be extern'd, as it is placed in a header
struct standard_deviation_subtract_mean_impl {
 public:
    template<typename InT, typename OutT>
    struct unary_function : public std::unary_function<InT, OutT> {
        __host__ __device__ OutT operator()(const InT &a) const;
    };
};
extern __constant__ double STANDARD_DEVIATION_MEAN;
extern std::mutex STANDARD_DEVIATION_MEAN_mutex;
extern standard_deviation_add_impl standard_deviation_add;
extern standard_deviation_subtract_mean_impl standard_deviation_subtract_mean;
template <typename OutT>
__device__ __forceinline__ OutT standard_deviation_add_impl::binary_function<OutT>::operator()(const OutT & a, const OutT & b) const {
    return a + b;
}
template<typename InT, typename OutT>
__device__ __forceinline__ OutT standard_deviation_subtract_mean_impl::unary_function<InT, OutT>::operator()(const InT &a) const {
    return pow(a - flamegpu_internal::STANDARD_DEVIATION_MEAN, 2.0);
}
}  // namespace flamegpu_internal

#endif  // INCLUDE_FLAMEGPU_SIM_AGENTLOGGINGCONFIG_REDUCTIONS_CUH_
