#pragma once

#include "puppet/retargeting/core/retargeting_plugin.hpp"

namespace puppet::retargeting {

    namespace gmr_wrap {
        class RetargeterHolder;
    }  // namespace gmr_wrap

    class GmrRetargetingPlugin final : public RetargetingPlugin {
       public:
        std::string name() const override { return "gmr"; }
        bool configure(const runtime::RuntimeConfig& config, std::string* error) override;
        bool process(const model::PrimitiveFrame& input, const std::string& bodyGroup, model::GroupControlIntent* output,
                     std::string* error) override;

       private:
        bool enabled_ = false;
        std::shared_ptr<gmr_wrap::RetargeterHolder> holder_;
    };

}  // namespace puppet::retargeting
