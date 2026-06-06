生成与导出说明

本目录包含两份 Mermaid 源文件：
 - `architecture_flowchart.mmd` — Cyber RT 模块交互流程图（flowchart）
本目录包含 Mermaid 源文件：
- `architecture.mmd` — Cyber RT 总体架构（序列图）
- `architecture_flowchart.mmd` — Cyber RT 模块交互流程图（flowchart）
- `transport_detail.mmd` — Transport 模块类/调用序列（sequenceDiagram）
- `transport_detail_flow.mmd` — Transport 模块类关系（flowchart）

如何将 `.mmd` 导出为 PNG（建议方法）：

1) 使用 `mermaid-cli`（需要 Node.js）

```bash
# 安装 mermaid-cli
npm install -g @mermaid-js/mermaid-cli

# 导出 PNG（在仓库根目录运行）
# architecture
mmdc -i docs/diagrams/architecture.mmd -o docs/diagrams/architecture.png
# transport 详细图
mmdc -i docs/diagrams/transport_detail.mmd -o docs/diagrams/transport_detail.png
# transport flowchart
mmdc -i docs/diagrams/transport_detail_flow.mmd -o docs/diagrams/transport_detail_flow.png
# architecture flowchart
mmdc -i docs/diagrams/architecture_flowchart.mmd -o docs/diagrams/architecture_flowchart.png
```

2) 使用 VS Code 的 Mermaid Preview 插件并右键导出为 PNG/SVG（交互式）

3) 使用在线渲染器或本地 Docker 镜像（如有防火墙或无 npm 环境）

注意与提示：
- Mermaid 渲染可能对 `sequenceDiagram` 中的大节点有布局差异，建议在导出前用预览检查并根据需要拆分或简化图。
- 若需要我代为生成 PNG（在此环境里可能无法直接渲染二进制图像），我可以尝试通过在线服务或提供一个 Dockerfile/脚本来在支持图形渲染的环境中生成并下载。
