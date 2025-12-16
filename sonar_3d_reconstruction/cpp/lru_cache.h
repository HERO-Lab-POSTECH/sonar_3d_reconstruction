#ifndef SONAR_3D_RECONSTRUCTION__LRU_CACHE_H_
#define SONAR_3D_RECONSTRUCTION__LRU_CACHE_H_

#include <list>
#include <unordered_map>
#include <optional>
#include <vector>
#include <functional>
#include <stdexcept>

namespace sonar_3d_reconstruction
{

/**
 * LRU (Least Recently Used) Cache
 * Template-based cache with configurable eviction callback
 *
 * @tparam Key Key type (must be hashable)
 * @tparam Value Value type (must be movable)
 * @tparam Hash Hash function for Key
 */
template<typename Key, typename Value, typename Hash = std::hash<Key>>
class LRUCache
{
public:
    using EvictionCallback = std::function<void(const Key&, Value&)>;

    /**
     * Constructor
     * @param max_size Maximum number of items in cache
     * @param on_eviction Callback when item is evicted (for saving)
     */
    explicit LRUCache(size_t max_size, EvictionCallback on_eviction = nullptr)
        : max_size_(max_size)
        , on_eviction_(on_eviction)
    {
        if (max_size == 0) {
            throw std::invalid_argument("LRUCache max_size must be > 0");
        }
    }

    ~LRUCache() = default;

    // Non-copyable
    LRUCache(const LRUCache&) = delete;
    LRUCache& operator=(const LRUCache&) = delete;

    // Movable
    LRUCache(LRUCache&&) = default;
    LRUCache& operator=(LRUCache&&) = default;

    /**
     * Get item from cache
     * @param key Key to look up
     * @return Pointer to value, or nullptr if not found
     * @note Moves item to front (most recently used)
     */
    Value* get(const Key& key)
    {
        auto it = map_.find(key);
        if (it == map_.end()) {
            return nullptr;
        }

        // Move to front (most recently used)
        items_.splice(items_.begin(), items_, it->second);
        return &(it->second->second);
    }

    /**
     * Get item from cache (const version)
     * @param key Key to look up
     * @return Pointer to value, or nullptr if not found
     * @note Does NOT update LRU order
     */
    const Value* peek(const Key& key) const
    {
        auto it = map_.find(key);
        if (it == map_.end()) {
            return nullptr;
        }
        return &(it->second->second);
    }

    /**
     * Check if key exists in cache
     * @param key Key to check
     * @return True if exists
     */
    bool contains(const Key& key) const
    {
        return map_.find(key) != map_.end();
    }

    /**
     * Put item in cache
     * @param key Key
     * @param value Value to store (will be moved)
     * @note If cache is full, evicts LRU item
     */
    void put(const Key& key, Value value)
    {
        auto it = map_.find(key);

        if (it != map_.end()) {
            // Update existing item
            it->second->second = std::move(value);
            items_.splice(items_.begin(), items_, it->second);
            return;
        }

        // Evict if necessary
        if (items_.size() >= max_size_) {
            evict_lru();
        }

        // Insert new item at front
        items_.emplace_front(key, std::move(value));
        map_[key] = items_.begin();
    }

    /**
     * Remove item from cache
     * @param key Key to remove
     * @return True if item was removed
     * @note Does NOT call eviction callback
     */
    bool remove(const Key& key)
    {
        auto it = map_.find(key);
        if (it == map_.end()) {
            return false;
        }

        items_.erase(it->second);
        map_.erase(it);
        return true;
    }

    /**
     * Evict least recently used item
     * @return Pair of (key, value) that was evicted, or nullopt if empty
     */
    std::optional<std::pair<Key, Value>> evict_lru()
    {
        if (items_.empty()) {
            return std::nullopt;
        }

        // Get LRU item (back of list)
        auto lru_it = std::prev(items_.end());
        Key key = lru_it->first;
        Value value = std::move(lru_it->second);

        // Call eviction callback before removal
        if (on_eviction_) {
            on_eviction_(key, value);
        }

        // Remove from map and list
        map_.erase(key);
        items_.erase(lru_it);

        return std::make_pair(std::move(key), std::move(value));
    }

    /**
     * Get all items in cache (most recent first)
     * @return Vector of pointers to values
     */
    std::vector<Value*> get_all()
    {
        std::vector<Value*> result;
        result.reserve(items_.size());

        for (auto& item : items_) {
            result.push_back(&item.second);
        }

        return result;
    }

    /**
     * Get all items in cache (const version)
     * @return Vector of const pointers to values
     */
    std::vector<const Value*> get_all() const
    {
        std::vector<const Value*> result;
        result.reserve(items_.size());

        for (const auto& item : items_) {
            result.push_back(&item.second);
        }

        return result;
    }

    /**
     * Get all keys in cache (most recent first)
     * @return Vector of keys
     */
    std::vector<Key> get_all_keys() const
    {
        std::vector<Key> result;
        result.reserve(items_.size());

        for (const auto& item : items_) {
            result.push_back(item.first);
        }

        return result;
    }

    /**
     * Apply function to all items
     * @param func Function taking (key, value&)
     */
    template<typename Func>
    void for_each(Func func)
    {
        for (auto& item : items_) {
            func(item.first, item.second);
        }
    }

    /**
     * Apply function to all items (const version)
     * @param func Function taking (key, const value&)
     */
    template<typename Func>
    void for_each(Func func) const
    {
        for (const auto& item : items_) {
            func(item.first, item.second);
        }
    }

    /**
     * Clear all items from cache
     * @param call_eviction If true, call eviction callback for each item
     */
    void clear(bool call_eviction = true)
    {
        if (call_eviction && on_eviction_) {
            for (auto& item : items_) {
                on_eviction_(item.first, item.second);
            }
        }

        items_.clear();
        map_.clear();
    }

    /**
     * Get current size of cache
     */
    size_t size() const { return items_.size(); }

    /**
     * Check if cache is empty
     */
    bool empty() const { return items_.empty(); }

    /**
     * Get maximum size of cache
     */
    size_t max_size() const { return max_size_; }

    /**
     * Set eviction callback
     * @param callback Function to call when item is evicted
     */
    void set_eviction_callback(EvictionCallback callback)
    {
        on_eviction_ = callback;
    }

    /**
     * Resize cache (evicts LRU items if necessary)
     * @param new_max_size New maximum size
     */
    void resize(size_t new_max_size)
    {
        if (new_max_size == 0) {
            throw std::invalid_argument("LRUCache max_size must be > 0");
        }

        while (items_.size() > new_max_size) {
            evict_lru();
        }

        max_size_ = new_max_size;
    }

private:
    size_t max_size_;
    EvictionCallback on_eviction_;

    // List of (key, value) pairs, ordered by recency (front = most recent)
    std::list<std::pair<Key, Value>> items_;

    // Map from key to iterator in list
    std::unordered_map<Key, typename std::list<std::pair<Key, Value>>::iterator, Hash> map_;
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION__LRU_CACHE_H_
