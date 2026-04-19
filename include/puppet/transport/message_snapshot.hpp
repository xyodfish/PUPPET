#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace puppet::runtime {

    class MessageBase {
       public:
        virtual ~MessageBase()                          = default;
        std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now();
        uint64_t sequenceId                             = 0;
    };

    template <typename T>
    class TypedMessage final : public MessageBase {
       public:
        explicit TypedMessage(std::shared_ptr<T> data_in) : data(std::move(data_in)) {}
        std::shared_ptr<T> data;
    };

    class MessageSnapshot {
       public:
        explicit MessageSnapshot(std::unordered_map<std::string, std::deque<std::shared_ptr<MessageBase>>> messages)
            : messages_(std::move(messages)) {}

        std::shared_ptr<MessageBase> getLatest(const std::string& topicName) const;
        std::vector<std::shared_ptr<MessageBase>> getAll(const std::string& topicName) const;
        std::vector<std::shared_ptr<MessageBase>> getIncrement(const std::string& topicName, uint64_t lastSequenceId,
                                                               uint64_t* newestSequenceId) const;

        template <typename T>
        std::shared_ptr<T> getLatestTyped(const std::string& topicName, int64_t* timestampNs) const {
            const auto msg = getLatest(topicName);
            if (msg == nullptr) {
                return nullptr;
            }
            const auto typed = std::dynamic_pointer_cast<TypedMessage<T>>(msg);
            if (typed == nullptr) {
                return nullptr;
            }
            if (timestampNs != nullptr) {
                *timestampNs = std::chrono::duration_cast<std::chrono::nanoseconds>(msg->timestamp.time_since_epoch()).count();
            }
            return typed->data;
        }

       private:
        std::unordered_map<std::string, std::deque<std::shared_ptr<MessageBase>>> messages_;
    };

    class MessageSnapshotManager {
       public:
        template <typename T>
        void updateMessage(const std::string& topicName, std::shared_ptr<T> data) {
            std::lock_guard<std::mutex> guard(mutex_);
            auto msg = std::make_shared<TypedMessage<T>>(std::move(data));
            updateInternal(topicName, std::move(msg));
        }

        MessageSnapshot getSnapshot();
        void setMaxHistorySize(const std::string& topicName, size_t maxSize);

       private:
        void updateInternal(const std::string& topicName, std::shared_ptr<MessageBase> message);

        std::mutex mutex_;
        std::unordered_map<std::string, std::deque<std::shared_ptr<MessageBase>>> messages_;
        std::unordered_map<std::string, size_t> maxHistorySizes_;
        uint64_t nextSequenceId_ = 0;
    };

}  // namespace puppet::runtime
