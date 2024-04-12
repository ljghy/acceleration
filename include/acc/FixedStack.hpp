#ifndef ACC_FIXED_STACK_HPP_
#define ACC_FIXED_STACK_HPP_

#include <cassert>
#include <type_traits>

#include <acc/Common.hpp>

namespace acc {

template <typename T, index_t N> class FixedStack {
private:
  template <typename T, bool B> struct ValOrRef {
    using type = const T &;
  };
  template <typename T> struct ValOrRef<T, true> {
    using type = T;
  };
  using ValOrRefT = ValOrRef<T, (sizeof(T) <= sizeof(T *) * 2 &&
                                 std::is_trivially_copyable<T>::value)>::type;

public:
  ACC_HOST_DEVICE FixedStack() = default;
  ACC_HOST_DEVICE FixedStack(index_t n) { assert(n <= N); };

  ACC_HOST_DEVICE ~FixedStack() = default;

  FixedStack(const FixedStack<T, N> &) = delete;
  FixedStack &operator=(const FixedStack<T, N> &) = delete;
  FixedStack(FixedStack<T, N> &&) = delete;
  FixedStack &operator=(FixedStack<T, N> &&) = delete;

  ACC_HOST_DEVICE T &top() { return m_data[m_size - 1]; }
  ACC_HOST_DEVICE ValOrRefT top() const { return m_data[m_size - 1]; }

  ACC_HOST_DEVICE bool empty() const { return m_size == 0; }

  ACC_HOST_DEVICE void push(ValOrRefT value) {
    assert(m_size < N);
    m_data[m_size++] = value;
  }

  ACC_HOST_DEVICE void pop() {
    assert(m_size > 0);
    --m_size;
  }

private:
  T m_data[N];
  index_t m_size = 0;
};

} // namespace acc

#endif
