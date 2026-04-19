#include "puppet/transport/message_snapshot.hpp"

namespace puppet::runtime {
    namespace {
        constexpr size_t kDefaultHistorySize = 5;
    }

    std::shared_ptr<MessageBase> MessageSnapshot::getLatest(const std::string& topicName) const {
        const auto it = messages_.find(topicName);
        if (it == messages_.end() || it->second.empty()) {
            return nullptr;
        }
        return it->second.back();
    }

    std::vector<std::shared_ptr<MessageBase>> MessageSnapshot::getAll(const std::string& topicName) const {
        std::vector<std::shared_ptr<MessageBase>> output;
        const auto it = messages_.find(topicName);
        if (it == messages_.end()) {
            return output;
        }
        output.assign(it->second.begin(), it->second.end());
        return output;
    }

    std::vector<std::shared_ptr<MessageBase>> MessageSnapshot::getIncrement(const std::string& topicName, uint64_t lastSequenceId,
                                                                            uint64_t* newestSequenceId) const {
        std::vector<std::shared_ptr<MessageBase>> output;
        const auto it = messages_.find(topicName);
        if (it == messages_.end()) {
            return output;
        }
        uint64_t latestSequenceId = lastSequenceId;
        for (const auto& msg : it->second) {
            if (msg->sequenceId > latestSequenceId) {
                latestSequenceId = msg->sequenceId;
            }
            if (msg->sequenceId > lastSequenceId) {
                output.push_back(msg);
            }
        }
        if (newestSequenceId != nullptr) {
            *newestSequenceId = latestSequenceId;
        }
        return output;
    }

    void MessageSnapshotManager::updateInternal(const std::string& topicName, std::shared_ptr<MessageBase> message) {
        ++nextSequenceId_;
        message->sequenceId = nextSequenceId_;
        auto& queue         = messages_[topicName];
        queue.push_back(std::move(message));

        const auto sizeIt  = maxHistorySizes_.find(topicName);
        const auto maxSize = sizeIt == maxHistorySizes_.end() ? kDefaultHistorySize : sizeIt->second;
        while (queue.size() > maxSize) {
            queue.pop_front();
        }
    }

    MessageSnapshot MessageSnapshotManager::getSnapshot() {
        std::lock_guard<std::mutex> guard(mutex_);
        return MessageSnapshot(messages_);
    }

    void MessageSnapshotManager::setMaxHistorySize(const std::string& topicName, size_t maxSize) {
        std::lock_guard<std::mutex> guard(mutex_);
        maxHistorySizes_[topicName] = maxSize;
        auto it                     = messages_.find(topicName);
        if (it == messages_.end()) {
            return;
        }
        while (it->second.size() > maxSize) {
            it->second.pop_front();
        }
    }

}  // namespace puppet::runtime
