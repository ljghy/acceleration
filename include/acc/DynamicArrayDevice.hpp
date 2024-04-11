#ifndef ACC_DYNAMIC_ARRAY_DEVICE_HPP_
#define ACC_DYNAMIC_ARRAY_DEVICE_HPP_

#include <acc/Common.hpp>

namespace acc {

template <typename T> class DynamicArrayDevice {
public:
  ACC_HOST_DEVICE DynamicArrayDevice(index_t initCapacity = 2)
      : m_data(new T[initCapacity]), m_size(), m_capacity(initCapacity) {}

  ACC_HOST_DEVICE ~DynamicArrayDevice() { delete[] m_data; }

  DynamicArrayDevice(const DynamicArrayDevice<T> &) = delete;
  DynamicArrayDevice &operator=(const DynamicArrayDevice<T> &) = delete;

  DynamicArrayDevice(DynamicArrayDevice<T> &&) = delete;
  DynamicArrayDevice &operator=(DynamicArrayDevice<T> &&) = delete;

  ACC_HOST_DEVICE T &operator[](index_t i) { return m_data[i]; }
  ACC_HOST_DEVICE const T &operator[](index_t i) const { return m_data[i]; }

  ACC_HOST_DEVICE T &back() { return m_data[m_size - 1]; }
  ACC_HOST_DEVICE const T &back() const { return m_data[m_size - 1]; }

  ACC_HOST_DEVICE bool empty() const { return m_size == 0; }
  ACC_HOST_DEVICE index_t size() const { return m_size; }

  ACC_HOST_DEVICE void push_back(const T &value) {
    if (m_size == m_capacity) {
      m_capacity = m_capacity == 0 ? 1 : 2 * m_capacity;
      T *newData = new T[m_capacity];
      for (index_t i = 0; i < m_size; ++i)
        newData[i] = m_data[i];
      delete[] m_data;
      m_data = newData;
    }
    m_data[m_size++] = value;
  }

  ACC_HOST_DEVICE void pop_back() {
    if (m_size > 0)
      --m_size;
  }

private:
  T *m_data = nullptr;
  index_t m_size = 0;
  index_t m_capacity = 0;
};

} // namespace acc

#endif
