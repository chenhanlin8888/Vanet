import pandas as pd
import matplotlib.pyplot as plt

# 指定列名
cols = ["runId", "sid", "Nraw", "S", "S_eff", "L", "qber", "keyRate", "protocol"]

# 读取 CSV
df = pd.read_csv("results.csv", header=None, names=cols)

print(f"✅ 已载入 CSV，共 {len(df)} 条记录\n")
print("前几行数据：")
print(df.head())

print("\n📊 数据描述:")
print(df.describe())

# 绘制 runId vs keyRate
plt.plot(df["runId"], df["keyRate"], "o-", color="blue")
plt.xlabel("Run ID")
plt.ylabel("Key Rate (L/Nraw)")
plt.title("Key Rate Across Runs")
plt.grid(True)
plt.savefig("keyrate_runs.png")
plt.close()

# 如果有多种协议（BB84 / SARG04）可以分组绘图
if "protocol" in df.columns:
    for proto, group in df.groupby("protocol"):
        plt.plot(group["runId"], group["keyRate"], "o-", label=proto)
    plt.xlabel("Run ID")
    plt.ylabel("Key Rate (L/Nraw)")
    plt.title("Key Rate Comparison")
    plt.legend()
    plt.grid(True)
    plt.savefig("keyrate_protocols.png")
    plt.close()


