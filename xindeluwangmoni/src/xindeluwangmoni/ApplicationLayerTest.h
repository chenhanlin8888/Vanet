// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include "xindeluwangmoni/xindeluwangmoni.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

namespace xindeluwangmoni {

// —— BB84 / SARG04 会话阶段 ——
enum Phase {
    HELLO=0, PREPARE=1, TRANSMIT=2, BASISB=3, SIFT=4, SAMPLE=5, QBER=6,
    EC=7, PA=8, DONE=9, ABORT=10,
    PAIR=11, CONCLUSIVE=12
};

// —— 会话状态 ——
struct Session {
    int sessionId{-1};
    int aliceId{-1};
    int bobId{-1};
    Phase phase{HELLO};

    int N{4096};
    int m{512};
    double qber{-1.0};
    double fEC{1.15};

    std::string bitsA;
    std::string basesA;
    std::string basesB;
    std::string bitsB;
    std::string siftMask;

    std::string sampleIdx;
    std::string sampleBitsA;

    std::string finalKey;

    cMessage* stepTimer{nullptr};
    simtime_t lastStep;

    // ===== SARG04 =====
    std::string statesA;
    std::string pairsA;
    std::string measB;
    std::string conclusiveMask;
    std::string bitsB_inferred;
    int S_conclusive{0};
    int sampleCount{0};
};

class XINDELUWANGMONI_API ApplicationLayerTest
    : public veins::DemoBaseApplLayer {

public:
    void initialize(int stage) override;
    void finish() override;

protected:
    // ===== 定时 =====
    simtime_t lastDroveAt;
    cMessage* scanTimer{nullptr};

    // ===== 参数 =====
    double v2vRadius{150.0};
    double baseErrorRate{0.01};
    double eavesdropRatio{0.0};
    double qberAlarm{0.11};
    double sampleFraction{0.125};
    double fEC{1.15};
    int    Nraw{4096};
    bool   useSARG04{false};

    // ===== 状态 =====
    std::unordered_map<int, Session> sessions;
    std::unordered_set<int> peersWithKey;
    bool hasKey{false};

    std::unordered_map<int, int>       activeWithPeer;
    std::unordered_map<int, simtime_t> cooldownUntil;

    // =========================================================
    // ===== MISSION-CRITICAL：新增状态与参数（不影响原逻辑）=====
    // =========================================================
    double missionMsgRate{5.0};     // msg / second
    int    missionKeyBits{128};     // bits per message
    long totalKeyBitsGenerated{0};

    std::unordered_map<int, int> keyPoolBits;   // peer -> available bits
    std::unordered_map<int, int> keyConsumed;   // peer -> consumed bits

    long missionMsgRequired{0};
    long missionMsgSupported{0};
    long missionOutageCount{0};

    simtime_t missionStartTime;
    simtime_t missionEndTime;

    cMessage* missionTimer{nullptr};

protected:
    // ===== Veins 回调 =====
    void onWSA(veins::DemoServiceAdvertisment*) override {}
    void onWSM(veins::BaseFrame1609_4* frame) override;
    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;

    // ===== 邻居与会话 =====
    int  findNearestVehicleIdWithin(double meters, double* outDist=nullptr);
    void tryStartBB84WithNearestPeer();

    // ===== 发送 =====
    void sendHello(Session& s, int dst);
    void sendPrepare(Session& s, int dst);
    void sendTransmit(Session& s, int dst);
    void sendBasesB(Session& s, int dst);
    void sendSift(Session& s, int dst);
    void sendSample(Session& s, int dst);
    void sendQBER(Session& s, int dst);
    void sendEC(Session& s, int dst);
    void sendPA(Session& s, int dst);
    void sendDone(Session& s, int dst);
    void sendAbort(Session& s, int dst, const char* reason);

    void sendPair(Session& s, int dst);
    void sendConclusive(Session& s, int dst);

    // ===== 逻辑 =====
    void generateAliceBitsAndBases(Session& s);
    void generateBobBasesAndMeasure(Session& s);
    void computeSiftAndSample(Session& s);

    void bobRandomMeasure(Session& s);
    void computeConclusiveAndSample(Session& s);

    void finalizeKeyAndColor(Session& s);
    void cleanupSession(int sid);

    static double H2(double p);
    void sendAbortSid(int sid, int dst, const char* reason);
};

} // namespace xindeluwangmoni
