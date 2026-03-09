import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;
import veins.base.utils.SimpleAddress;
import veins.modules.application.traci.TraCIDemo11pMessage;

namespace xindeluwangmoni;

packet BB84Message extends veins::BaseFrame1609_4 {
    int    phase;         // 0:HELLO,1:PREPARE,2:TRANSMIT,3:BASISB,4:SIFT,5:SAMPLE,6:QBER,7:EC,8:PA,9:DONE,10:ABORT,
                          // 11:PAIR(Alice->Bob 公布候选态对), 12:CONCLUSIVE(Bob->Alice 返回可判定掩码)
    int    sessionId;
    int    srcId;
    int    dstId;

    int    N;
    int    m;
    double qber;
    double fecFactor;

    string bitsA;         // "010101..."（BB84：按基映射；SARG04：按“单态→比特”映射）
    string basesA;        // "ZZXXZ..."（BB84 用）
    string basesB;        // ★ Bob 的基（BASISB 阶段发送，BB84 用）
    string siftMask;      // "101001..."（BB84：同基掩码；SARG04 可复用为“可判定掩码”亦可）
    string sampleMask;    // （可留作他用）

    string sampleIdx;     // ★ 例如 "3,7,11"
    string sampleBitsA;   // ★ 与 sampleIdx 对齐，例如 "101"

    string ecParity;      // （可选）
    string paDigest;      // （可选）

    // ====== 追加：SARG04 专用字段（仅新增，不影响既有接收端） ======
    string statesA;        // Alice 逐位发送的单态串：字符集 {'0','1','+','-'}
    string pairsA;         // Alice 逐位公布的候选对索引串：字符集 {'0','1','2','3'}
                           // 约定：P0={|0>,|+>}, P1={|0>,|->}, P2={|1>,|+>}, P3={|1>,|->}
    string conclusiveMask; // Bob 逐位可判定掩码：'1' 表示该位可判定（SARG04）
    string measB;          // （可选）Bob 在自身基下的二值测量结果串：'0','1'（通常不需要发，仅调试）
    string bitsBInfer;     // （可选）Bob 在可判定位推断出的比特串（不可判定位可用 '?'）
}
