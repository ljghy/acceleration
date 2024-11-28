#ifndef ACC_STATIC_STACK_HPP_
#define ACC_STATIC_STACK_HPP_

#include <cassert>
#include <type_traits>

#include <acc/Config.hpp>

namespace acc {

template <typename T, int N, typename = std::enable_if_t<std::is_trivial_v<T>>>
class StaticStack {
public:
  ACC_HOST_DEVICE StaticStack() : m_top(0) {}
  ACC_HOST_DEVICE StaticStack(const int n) { assert(n <= N); };
  ACC_HOST_DEVICE ~StaticStack() = default;

  StaticStack(const StaticStack<T, N> &) = delete;
  StaticStack &operator=(const StaticStack<T, N> &) = delete;
  StaticStack(StaticStack<T, N> &&) = delete;
  StaticStack &operator=(StaticStack<T, N> &&) = delete;

  ACC_HOST_DEVICE T &top() { return m_data[m_top - 1]; }
  ACC_HOST_DEVICE T top() const { return m_data[m_top - 1]; }

  ACC_HOST_DEVICE bool empty() const { return m_top == 0; }

  ACC_HOST_DEVICE int size() const { return m_top; }

  ACC_HOST_DEVICE void push(const T value) {
    assert(m_size < N);
    m_data[m_top++] = value;
  }

  ACC_HOST_DEVICE void emplace(const T value) {
    assert(m_size < N);
    m_data[m_top++] = value;
  }

  ACC_HOST_DEVICE void pop() {
    assert(m_size > 0);
    --m_top;
  }

private:
  T m_data[N];
  int m_top;
};

} // namespace acc

#endif
