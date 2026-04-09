# test 目录约定

- `test/unit/`：单元测试（建议优先覆盖 core logic）。
- `test/integration/`：集成测试（覆盖模块协作链路）。
- `test/demos/`：轻量验证 demo（验证某个想法、输入链路或接口，不作为正式产品入口）。

放置原则：

- 需要长期对外展示、可作为产品入口的 demo，请放在 `app/cpp/demos` 或 `app/python/demos`。
- 仅用于研发阶段验证、可快速迭代的小 demo，可以放在 `test/demos`。
