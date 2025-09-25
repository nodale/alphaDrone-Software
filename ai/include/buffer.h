#pragma once
#include <atomic>
#include <bit>
#include <concepts>
#include <expected>
#include <stdexcept>
#include <system_error>
#include <vector>

template <typename T>
concept StorageProviderConcept = requires(T t, size_t i) {
  { t[i] } -> std::convertible_to<typename T::value_type &>;
  { t.capacity() } -> std::convertible_to<size_t>;
  std::is_constructible_v<T, size_t>;
};

template <typename T, StorageProviderConcept StorageProvider = std::vector<T>>
class SingleProducerConsumerQueue {
public:
  SingleProducerConsumerQueue(size_t len)
      : storage_provider_(std::bit_ceil(len)) {};

  bool enqueue(T &&item) {
    size_t current_tail = tail_.load(std::memory_order_relaxed);
    size_t next_tail = (current_tail + 1) & (storage_provider_.capacity() - 1);
    if (next_tail == head_.load(std::memory_order_acquire))
      return false;

    storage_provider_[current_tail] = std::move(item);
    tail_.store(next_tail, std::memory_order_release);
    return true;
  }

  std::expected<T *, std::errc> top() {
    size_t current_head = head_.load(std::memory_order_relaxed);
    if (current_head == tail_.load(std::memory_order_acquire))
      throw std::runtime_error("Queue is empty");
    return &storage_provider_[current_head];
  }

  void dequeue() {
    size_t current_head = head_.load(std::memory_order_relaxed);
    if (current_head == tail_.load(std::memory_order_acquire))
      throw std::runtime_error("Queue is empty");
    head_.store((current_head + 1) & (storage_provider_.capacity() - 1),
                std::memory_order_release);
  }

private:
  StorageProvider storage_provider_;
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};
};
