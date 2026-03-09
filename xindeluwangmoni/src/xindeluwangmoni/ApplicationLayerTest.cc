// SPDX-License-Identifier: GPL-2.0-or-later
#include "xindeluwangmoni/ApplicationLayerTest.h"
#include "veins/base/utils/FindModule.h"
#include "veins/base/utils/SimpleAddress.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "xindeluwangmoni/BB84Message_m.h"
#include <fstream>
#include <algorithm>
#include <cmath>
#include <sstream>

using namespace veins;
using namespace xindeluwangmoni;

Define_Module(xindeluwangmoni::ApplicationLayerTest);

// ========= 工具 =========
double ApplicationLayerTest::H2(double p) {
    if (p <= 0.0 || p >= 1.0) return 0.0;
    return -p*log2(p) - (1.0-p)*log2(1.0-p);
}

// ========= 初始化 =========
void ApplicationLayerTest::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        lastDroveAt = simTime();

        if (hasPar("v2vRadius"))        v2vRadius      = par("v2vRadius").doubleValue();
        if (hasPar("baseErrorRate"))    baseErrorRate  = par("baseErrorRate").doubleValue();
        if (hasPar("eavesdropRatio"))   eavesdropRatio = par("eavesdropRatio").doubleValue();
        if (hasPar("qberAlarm"))        qberAlarm      = par("qberAlarm").doubleValue();
        if (hasPar("sampleFraction"))   sampleFraction = par("sampleFraction").doubleValue();
        if (hasPar("fEC"))              fEC            = par("fEC").doubleValue();
        if (hasPar("Nraw"))             Nraw           = par("Nraw").intValue();
        if (hasPar("useSARG04"))        useSARG04      = par("useSARG04").boolValue();

        hasKey = false;
        scanTimer = new cMessage("scanTimer");
        scheduleAt(simTime() + 0.5, scanTimer);

        EV_INFO << "[APP] INIT"
                << " Nraw=" << Nraw
                << " v2vRadius=" << v2vRadius
                << " baseErr=" << baseErrorRate
                << " eaves=" << eavesdropRatio
                << " qberAlarm=" << qberAlarm
                << " fEC=" << fEC
                << " useSARG04=" << (useSARG04 ? "true" : "false")
                << "\n";



        // ===== MISSION-CRITICAL: init =====
        missionStartTime = simTime();
        missionTimer = new cMessage("missionTimer");
        scheduleAt(simTime() + 1.0, missionTimer);

        EV_INFO << "[MISSION] INIT"
                << " msgRate=" << missionMsgRate
                << " keyPerMsg=" << missionKeyBits
                << "\n";

    }
}

void ApplicationLayerTest::handlePositionUpdate(cObject* obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);
}

// ========= 近邻选择 =========
int ApplicationLayerTest::findNearestVehicleIdWithin(double meters, double* outDist) {
    cModule* root = getSimulation()->getSystemModule();
    TraCIMobility* myMob = FindModule<TraCIMobility*>::findSubModule(findHost());
    if (!myMob) return -1;

    Coord me = myMob->getPositionAt(simTime());
    int bestId = -1; double bestD = 1e100;

    for (cModule::SubmoduleIterator it(root); !it.end(); ++it) {
        cModule* host = *it;
        if (!host || host==findHost()) continue;

        auto* applMod = host->getSubmodule("appl");
        auto* appl = dynamic_cast<ApplicationLayerTest*>(applMod);
        if (!appl) continue;

        auto* vmob = dynamic_cast<TraCIMobility*>(host->getSubmodule("veinsmobility"));
        if (!vmob) continue;

        double d = me.distance(vmob->getPositionAt(simTime()));
        if (d <= meters) {
            int vid = (int)appl->myId;
            if (!peersWithKey.count(vid)) {
                if (d < bestD) { bestD = d; bestId = vid; }
            }
        }
    }
    if (outDist) *outDist = (bestId<0? -1.0 : bestD);
    return bestId;
}

// ========= 主动发起 =========
void ApplicationLayerTest::tryStartBB84WithNearestPeer() {
    double d = 0;
    int peer = findNearestVehicleIdWithin(v2vRadius, &d);
    if (peer < 0) return;

    // 冷却 & 占用检查
    if (cooldownUntil.count(peer) && simTime() < cooldownUntil[peer]) return;
    if (activeWithPeer.count(peer)) return;

    // 发起方仲裁：较小ID作为 Alice
    if ((int)myId > peer) {
        cooldownUntil[peer] = simTime() + 1.0; // 等对端发起
        return;
    }

    Session s;
    s.sessionId = intrand(INT_MAX);
    s.aliceId   = (int)myId;
    s.bobId     = peer;
    s.phase     = HELLO;
    s.N         = Nraw;
    s.m         = std::max(1, (int)std::round(s.N * sampleFraction));
    s.fEC       = fEC;

    sessions[s.sessionId] = s;
    activeWithPeer[peer]  = s.sessionId;

    EV_INFO << "[APP] start session sid=" << s.sessionId
            << " A=" << s.aliceId << " B=" << s.bobId
            << " dist=" << d << "m\n";

    sendHello(sessions[s.sessionId], peer);

    sessions[s.sessionId].stepTimer = new cMessage(("bb84_"+std::to_string(s.sessionId)).c_str());
    scheduleAt(simTime() + 0.02, sessions[s.sessionId].stepTimer);

    cooldownUntil[peer] = simTime() + 5.0;
}

// ========= onWSM =========
void ApplicationLayerTest::onWSM(BaseFrame1609_4* frame) {
    auto* m = dynamic_cast<BB84Message*>(frame);
    if (!m) return;

    if (m->getDstId() != (int)myId && m->getDstId() != -1) return;

    const int sid = m->getSessionId();
    const int src = m->getSrcId();
    const int dst = m->getDstId();
    const int iph = m->getPhase();
    Phase ph = (iph>=HELLO && iph<=ABORT) ? (Phase)iph : ABORT;
    if (iph == PAIR) ph = PAIR;
    if (iph == CONCLUSIVE) ph = CONCLUSIVE;

    EV_INFO << "[APP] RECV phase=" << iph
            << " sid=" << sid
            << " src=" << src
            << " dst=" << dst
            << " myId=" << (int)myId
            << " hasSession=" << (sessions.find(sid)!=sessions.end()) << "\n";

    auto abortBySid = [&](int sidX, int peer, const char* reason) {
        Session tmp; tmp.sessionId = sidX; tmp.aliceId = peer; tmp.bobId = (int)myId;
        sendAbort(tmp, peer, reason);
    };

    auto getDistanceTo = [&](int vehId, double* out) -> bool {
        Coord myp, peerp; bool okMy=false, okPeer=false;
        if (auto* tm = veins::FindModule<veins::TraCIMobility*>::findSubModule(findHost())) {
            myp = tm->getPositionAt(simTime()); okMy = true;
        }
        for (cModule::SubmoduleIterator it(getSimulation()->getSystemModule()); !it.end(); ++it) {
            cModule* host = *it; if (!host) continue;
            auto* appl = dynamic_cast<ApplicationLayerTest*>(host->getSubmodule("appl"));
            if (!appl || (int)appl->myId != vehId) continue;
            if (auto* vmob = dynamic_cast<TraCIMobility*>(host->getSubmodule("veinsmobility"))) {
                peerp = vmob->getPositionAt(simTime()); okPeer = true;
            }
            break;
        }
        if (okMy && okPeer) { if (out) *out = myp.distance(peerp); return true; }
        return false;
    };

    auto it = sessions.find(sid);
    if (it != sessions.end()) it->second.lastStep = simTime();

    // ------- 1) Bob 收 HELLO → 接受 -------
    if (it == sessions.end() && ph == HELLO && dst == (int)myId) {
        double d = 0;
        if (getDistanceTo(src, &d) && d > v2vRadius) {
            EV_INFO << "[APP] reject HELLO sid="<<sid<<" from "<<src<<" dist="<<d<<"m\n";
            abortBySid(sid, src, "distance>radius");
            return;
        }

        // 碰撞/去重
        auto itAct = activeWithPeer.find(src);
        if (itAct != activeWithPeer.end()) {
            int sidLocal = itAct->second;
            if (sidLocal != sid) {
                bool keepIncoming = (src < (int)myId);
                if (!keepIncoming) { abortBySid(sid, src, "dup/collision"); return; }
                cleanupSession(sidLocal);
                activeWithPeer.erase(itAct);
            }
        }

        Session s;
        s.sessionId = sid;
        s.aliceId   = src;
        s.bobId     = (int)myId;
        s.N         = (m->getN()>0 ? m->getN() : Nraw);
        s.m         = std::max(1, (int)std::round(s.N * sampleFraction));
        s.fEC       = (m->getFecFactor()>0 ? m->getFecFactor() : fEC);
        s.phase     = PREPARE;
        s.lastStep  = simTime();
        s.stepTimer = new cMessage(("bb84_"+std::to_string(sid)).c_str());
        sessions[sid] = s;

        activeWithPeer[src] = sid;
        cooldownUntil[src]  = simTime() + 5.0;

        EV_INFO << "[APP] accept session sid="<<sid<<" as Bob\n";
        sendPrepare(sessions[sid], src);
        scheduleAt(simTime() + 0.02, sessions[sid].stepTimer);
        return;
    }

    if (it == sessions.end()) return;
    Session& s = it->second;

    switch (ph) {
        case PREPARE:
            if (s.phase == PREPARE || s.phase == HELLO) {
                s.phase = TRANSMIT;
                if ((int)myId == s.aliceId) {
                    // 不改主调用：内部根据 useSARG04 决定生成何种内容
                    generateAliceBitsAndBases(s);
                    sendTransmit(s, s.bobId);
                }
            }
            break;

        case TRANSMIT: {
            if ((int)myId == s.bobId) {
                // 共用：把 Alice 侧携带的信息收下来
                s.bitsA   = m->getBitsA();     // BB84 或 SARG04 均保存
                s.basesA  = m->getBasesA();    // 仅 BB84 用
                s.statesA = m->getStatesA();   // 仅 SARG04 用

                if (useSARG04) {
                    // SARG04：Bob 本地随机基测量（不向 Alice 发送 basesB）
                    bobRandomMeasure(s);
                    s.phase = TRANSMIT; // 等待 PAIR
                } else {
                    // BB84 原路径
                    generateBobBasesAndMeasure(s);
                    sendBasesB(s, s.aliceId);
                    s.phase = BASISB;
                }
            }
            break;
        }

        // ========== SARG04：PAIR（Alice->Bob 公布候选态对） ==========
        case PAIR:
            if ((int)myId == s.bobId) {
                s.pairsA = m->getPairsA();

                // 基于 pairsA + (basesB, measB) 计算 conclusiveMask 与 bitsB_inferred
                s.conclusiveMask.assign(s.N, '0');
                s.bitsB_inferred.assign(s.N, '?');
                auto zOpp = [&](char z)->char{ return (z=='0')? '1':'0'; };
                auto xOpp = [&](char x)->char{ return (x=='+')? '-':'+'; };

                for (int i=0;i<s.N;i++) {
                    // pair 映射：P0={0,+}, P1={0,-}, P2={1,+}, P3={1,-}
                    int pid = (i < (int)s.pairsA.size()) ? (s.pairsA[i]-'0') : 0;
                    char zS = (pid<=1? '0':'1');
                    char xS = (pid%2==0? '+':'-');

                    bool concl = false;
                    char inferredState = '?';

                    char b = (i < (int)s.basesB.size()) ? s.basesB[i] : 'Z';
                    char mb= (i < (int)s.measB.size())  ? s.measB[i]  : '0';

                    if (b=='Z') {
                        // Z基：若测到 zOpp(zS) ⇒ 排除 zS ⇒ 一定位于 xS
                        if (mb == zOpp(zS)) { concl=true; inferredState = xS; }
                    } else {
                        // X基：若测到 xOpp(xS) ⇒ 排除 xS ⇒ 一定位于 zS
                        char xOppBit = (xS=='+'? '1':'0'); // '+'→0, '−'→1
                        if (mb == xOppBit) { concl=true; inferredState = zS; }
                    }

                    if (concl) {
                        s.conclusiveMask[i] = '1';
                        s.bitsB_inferred[i] = (inferredState=='0'||inferredState=='+') ? '0' : '1';
                    }
                }

                // 统计可判定位
                s.S_conclusive = 0; for (char c: s.conclusiveMask) if (c=='1') ++s.S_conclusive;

                // 发送可判定掩码（使用新相位 CONCLUSIVE；同时为兼容也写进 siftMask）
                sendConclusive(s, s.aliceId);
                s.phase = SIFT; // 或 CONCLUSIVE
            }
            break;

        // ========== 旧 BB84：BASISB ==========
        case BASISB:
            if ((int)myId == s.aliceId) {
                s.basesB = m->getBasesB();
                computeSiftAndSample(s);     // BB84：同基集合抽样
                sendSift(s, s.bobId);
                sendSample(s, s.bobId);
                s.phase = SAMPLE;
            }
            break;

        // ========== 旧 BB84：SIFT（Alice->Bob） ==========
        case SIFT:
            if ((int)myId == s.bobId) {
                s.siftMask = m->getSiftMask();
                EV_INFO << "[APP] recv SIFT sid="<<sid<<" from "<<src<<"\n";
            }
            break;

        // ========== SARG04：CONCLUSIVE（Bob->Alice） ==========
        case CONCLUSIVE:
            if ((int)myId == s.aliceId) {
                s.conclusiveMask = m->getConclusiveMask();
                s.siftMask       = s.conclusiveMask; // 兼容复用
                computeConclusiveAndSample(s);       // 仅在可判定集合内抽样
                sendSample(s, s.bobId);
                s.phase = SAMPLE;
            }
            break;

        case SAMPLE:
            if ((int)myId == s.bobId) {
                // 解析 sampleIdx
                std::string idxStr = m->getSampleIdx();
                std::string bitsA  = m->getSampleBitsA();
                std::vector<int> idx; idx.reserve(bitsA.size());
                int val=0; bool inNum=false;
                for (char ch : idxStr) {
                    if (ch>='0' && ch<='9') { val = val*10 + (ch-'0'); inNum = true; }
                    else if (inNum) { idx.push_back(val); val=0; inNum=false; }
                }
                if (inNum) idx.push_back(val);

                int M = (int)idx.size();
                int errors = 0;

                bool isS = useSARG04;
                for (int k=0; k<M; ++k) {
                    int i = idx[k];
                    bool aBit = (k < (int)bitsA.size()) ? (bitsA[k]=='1') : false;
                    bool bBit;
                    if (isS) {
                        bBit = (i < (int)s.bitsB_inferred.size()) ? (s.bitsB_inferred[i]=='1') : false;
                    } else {
                        bBit = (i < (int)s.bitsB.size()) ? (s.bitsB[i]=='1') : false;
                    }
                    if (aBit != bBit) ++errors;
                }
                s.qber = (M>0) ? (double)errors/M : 0.5;
                EV_INFO << "[APP] SAMPLE->QBER sid="<<sid<<" M="<<M<<" q="<<s.qber<<"\n";
                sendQBER(s, s.aliceId);
                s.phase = QBER;
            }
            break;

        case QBER:
            if ((int)myId == s.aliceId) {
                s.qber = m->getQber();
                EV_INFO << "[APP] recv QBER sid="<<sid<<" q="<<s.qber<<" from "<<src<<"\n";
                if (s.qber > qberAlarm) {
                    sendAbort(s, s.bobId, "QBER too high");
                    s.phase = ABORT;
                    cleanupSession(s.sessionId);
                } else {
                    sendEC(s, s.bobId); // 进入 EC → PA → DONE
                    s.phase = EC;
                }
            }
            break;

        case EC:
            EV_INFO << "[APP] recv EC sid="<<sid<<" from "<<src<<"\n";
            sendPA(s, ((int)myId==s.aliceId? s.bobId : s.aliceId));
            s.phase = PA;
            break;

        case PA:
            EV_INFO << "[APP] recv PA sid="<<sid<<" from "<<src<<"\n";
            finalizeKeyAndColor(s);
            sendDone(s, ((int)myId==s.aliceId? s.bobId : s.aliceId));
            s.phase = DONE;
            cleanupSession(s.sessionId);
            break;

        case DONE:
            EV_INFO << "[APP] recv DONE sid="<<sid<<" from "<<src<<"\n";
            cleanupSession(s.sessionId);
            break;

        case HELLO: {
            // 重复 HELLO 的去重（保留原逻辑）
            int peerKey = (src == s.aliceId ? s.aliceId : s.bobId);
            auto itAct  = activeWithPeer.find(peerKey);
            if (itAct != activeWithPeer.end() && sid != itAct->second) {
                bool keepIncoming = (src < (int)myId);
                if (!keepIncoming) abortBySid(sid, src, "dup/collision");
                else {
                    cleanupSession(itAct->second);
                    activeWithPeer.erase(itAct);
                }
            }
            break;
        }

        case ABORT:
            EV_WARN << "[APP] recv ABORT sid="<<sid<<" from "<<src<<"\n";
            cleanupSession(sid);
            break;

        default: break;
    }
}

void ApplicationLayerTest::handleSelfMsg(cMessage* msg) {

    // ===== MISSION-CRITICAL: periodic key consumption =====
    if (msg == missionTimer) {

        // --- 关键修正 1：没有密钥时不计入需求 / outage ---
        if (keyPoolBits.empty()) {
            EV_INFO << "[MISSION][TICK] t=" << simTime()
                    << " no-key, skip\n";
            scheduleAt(simTime() + 1.0, missionTimer);
            return;
        }

        int required = (int)std::round(missionMsgRate);
        missionMsgRequired += required;

        int supported = 0;
        int outage   = 0;

        for (int i = 0; i < required; ++i) {
            bool ok = false;

            for (auto& kv : keyPoolBits) {
                int peer = kv.first;
                int& pool = kv.second;

                if (pool >= missionKeyBits) {
                    pool -= missionKeyBits;
                    keyConsumed[peer] += missionKeyBits;
                    missionMsgSupported++;
                    supported++;
                    ok = true;

                    EV_DEBUG << "[MISSION][USE]"
                             << " peer=" << peer
                             << " consume=" << missionKeyBits
                             << " remain=" << pool
                             << " t=" << simTime()
                             << "\n";
                    break;
                }
            }

            if (!ok) {
                missionOutageCount++;
                outage++;
            }
        }

        EV_INFO << "[MISSION][TICK]"
                << " t=" << simTime()
                << " required=" << required
                << " supported=" << supported
                << " outage=" << outage
                << " peers=" << keyPoolBits.size()
                << "\n";

        scheduleAt(simTime() + 1.0, missionTimer);
        return;
    }

    // ===== 原有扫描逻辑 =====
    if (msg == scanTimer) {
        tryStartBB84WithNearestPeer();
        scheduleAt(simTime() + 0.5, scanTimer);
        return;
    }

    // ===== 会话 stepTimer =====
    Session* ps = nullptr;
    for (auto it2 = sessions.begin(); it2 != sessions.end(); ++it2) {
        if (it2->second.stepTimer == msg) { ps = &it2->second; break; }
    }
    if (ps) {
        Session& s = *ps;
        if (s.phase == DONE || s.phase == ABORT) {
            cleanupSession(s.sessionId);
            return;
        }
        scheduleAt(simTime() + 0.2, s.stepTimer);
        return;
    }

    DemoBaseApplLayer::handleSelfMsg(msg);
}
// ========= 发送 =========
void ApplicationLayerTest::sendHello(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(HELLO);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    m->setN(s.N);
    m->setFecFactor(s.fEC);
    EV_INFO << "[APP] SEND HELLO sid="<<s.sessionId<<" dst="<<dst<<"\n";
    sendDown(m);
}
void ApplicationLayerTest::sendPrepare(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(PREPARE);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    EV_INFO << "[APP] SEND PREPARE sid="<<s.sessionId<<" dst="<<dst<<"\n";
    sendDown(m);
}
void ApplicationLayerTest::sendTransmit(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(TRANSMIT);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    m->setN(s.N);
    m->setFecFactor(s.fEC);

    // BB84 常规字段
    m->setBasesA(s.basesA.c_str());
    m->setBitsA(s.bitsA.c_str());

    // SARG04：若开启，附带 statesA，并在随后发送 PAIR
    if (useSARG04 && !s.statesA.empty())
        m->setStatesA(s.statesA.c_str());

    EV_INFO << "[APP] SEND TRANSMIT sid="<<s.sessionId<<" dst="<<dst
            <<" N="<<s.N<<"\n";
    sendDown(m);

    if (useSARG04 && !s.pairsA.empty()) {
        sendPair(s, dst);
    }
}
void ApplicationLayerTest::sendBasesB(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(BASISB);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    m->setBasesB(s.basesB.c_str());
    EV_INFO << "[APP] SEND BASISB sid="<<s.sessionId<<" dst="<<" "<<dst<<"\n";
    sendDown(m);
}
void ApplicationLayerTest::sendSift(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(SIFT);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    m->setSiftMask(s.siftMask.c_str());
    EV_INFO << "[APP] SEND SIFT sid="<<s.sessionId<<" dst="<<dst<<"\n";
    sendDown(m);
}
void ApplicationLayerTest::sendSample(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(SAMPLE);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    m->setSampleIdx(s.sampleIdx.c_str());
    m->setSampleBitsA(s.sampleBitsA.c_str());
    EV_INFO << "[APP] SEND SAMPLE sid="<<s.sessionId<<" dst="<<dst
            <<" M="<<s.sampleBitsA.size()<<"\n";
    sendDown(m);
}
void ApplicationLayerTest::sendQBER(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(QBER);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    m->setQber(s.qber);
    EV_INFO << "[APP] SEND QBER sid="<<s.sessionId<<" q="<<s.qber<<" dst="<<dst<<"\n";
    sendDown(m);
}
void ApplicationLayerTest::sendEC(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(EC);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    m->setFecFactor(s.fEC);
    EV_INFO << "[APP] SEND EC sid="<<s.sessionId<<" dst="<<dst<<"\n";
    sendDown(m);
}
void ApplicationLayerTest::sendPA(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(PA);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    m->setPaDigest("digest"); // TODO: 真实哈希
    EV_INFO << "[APP] SEND PA sid="<<s.sessionId<<" dst="<<dst<<"\n";
    sendDown(m);
}
void ApplicationLayerTest::sendDone(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(DONE);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    EV_INFO << "[APP] SEND DONE sid="<<s.sessionId<<" dst="<<dst<<"\n";
    sendDown(m);
}
void ApplicationLayerTest::sendAbort(Session& s, int dst, const char* reason) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(ABORT);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    EV_WARN << "[APP] ABORT sid="<<s.sessionId<<" reason="<<reason<<"\n";
    sendDown(m);
}

// ===== SARG04 新增：发送 PAIR/CONCLUSIVE =====
void ApplicationLayerTest::sendPair(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(PAIR);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    m->setPairsA(s.pairsA.c_str());
    EV_INFO << "[APP] SEND PAIR sid="<<s.sessionId<<" dst="<<dst<<"\n";
    sendDown(m);
}
void ApplicationLayerTest::sendConclusive(Session& s, int dst) {
    auto* m = new BB84Message();
    populateWSM(m);
    m->setPhase(CONCLUSIVE);
    m->setSessionId(s.sessionId);
    m->setSrcId((int)myId);
    m->setDstId(dst);
    m->setConclusiveMask(s.conclusiveMask.c_str());
    // 兼容：也同步进 siftMask（便于旧日志/工具复用）
    m->setSiftMask(s.conclusiveMask.c_str());
    EV_INFO << "[APP] SEND CONCLUSIVE sid="<<s.sessionId<<" dst="<<dst<<"\n";
    sendDown(m);
}

// ========= 逻辑（BB84 原有 + SARG04 开关）=========
void ApplicationLayerTest::generateAliceBitsAndBases(Session& s) {
    if (useSARG04) {
        // === SARG04：生成 statesA/pairsA 与 bitsA（按单态映射）
        s.statesA.resize(s.N);
        s.pairsA.resize(s.N);
        s.bitsA.resize(s.N);

        for (int i=0;i<s.N;i++) {
            int r = intrand(4);
            char st = (r==0? '0' : r==1? '1' : r==2? '+' : '-');
            s.statesA[i] = st;

            // pair：P0={0,+}, P1={0,-}, P2={1,+}, P3={1,-}
            int pairIdx;
            if (st=='0') pairIdx = (intrand(2)? 0:1);
            else if (st=='1') pairIdx = (intrand(2)? 2:3);
            else if (st=='+') pairIdx = (intrand(2)? 0:2);
            else              pairIdx = (intrand(2)? 1:3);
            s.pairsA[i] = '0' + pairIdx;

            // 单态→比特映射
            s.bitsA[i] = (st=='0' || st=='+') ? '0' : '1';
        }
        s.basesA.clear(); // 不公布基
    } else {
        // === BB84：生成 bitsA/basesA（原逻辑）
        s.bitsA.resize(s.N);
        s.basesA.resize(s.N);
        for (int i=0;i<s.N;i++) {
            s.bitsA[i]  = (intrand(2)? '1':'0');
            s.basesA[i] = (intrand(2)? 'X':'Z');
        }
        s.statesA.clear();
        s.pairsA.clear();
    }
}

void ApplicationLayerTest::generateBobBasesAndMeasure(Session& s) {
    // === BB84 原路径 ===
    double p0 = baseErrorRate;
    double pe = eavesdropRatio;
    double p  = 1.0 - (1.0 - p0) * (1.0 - pe);

    s.basesB.resize(s.N);
    s.bitsB.resize(s.N);

    for (int i=0;i<s.N;i++) {
        s.basesB[i] = (intrand(2)? 'X':'Z');
        bool bBit;
        if (s.basesA.size()==(size_t)s.N && s.bitsA.size()==(size_t)s.N) {
            bool aBit = (s.bitsA[i]=='1');
            if (s.basesA[i]==s.basesB[i]) {
                bool flip = (uniform(0,1) < p);
                bBit = flip ? !aBit : aBit;
            } else {
                bBit = (intrand(2)!=0);
            }
        } else {
            bBit = (intrand(2)!=0);
        }
        s.bitsB[i] = bBit ? '1':'0';
    }
}

void ApplicationLayerTest::computeSiftAndSample(Session& s) {
    s.siftMask.resize(s.N);
    std::vector<int> idx; idx.reserve(s.N);
    for (int i=0; i<s.N; i++) {
        bool same = (s.basesA.size()==(size_t)s.N &&
                     s.basesB.size()==(size_t)s.N &&
                     s.basesA[i]==s.basesB[i]);
        s.siftMask[i] = same ? '1':'0';
        if (same) idx.push_back(i);
    }
    if ((int)idx.size() < 2) {
        sendAbort(s, s.bobId, "sift too short");
        s.phase = ABORT;
        return;
    }

    // 修正：M 按 sift 后集合大小 × sampleFraction
    int M = std::max(1, (int)std::round(idx.size() * sampleFraction));

    std::string idxStr, bitsStr;
    idxStr.reserve(4*M);
    bitsStr.reserve(M);

    for (int k=0; k<M; k++) {
        int i = idx[intrand(idx.size())]; // 有放回采样
        if (k) idxStr.push_back(',');
        idxStr += std::to_string(i);
        bitsStr.push_back(s.bitsA[i]);
    }
    s.sampleIdx   = idxStr;
    s.sampleBitsA = bitsStr;
    s.sampleCount = M;
}


// ========= 逻辑（SARG04 新增）=========
void ApplicationLayerTest::bobRandomMeasure(Session& s) {
    // 产生 Bob 的随机基与二值测量结果（SARG04）
    double p0 = baseErrorRate, pe = eavesdropRatio;
    double p  = 1.0 - (1.0 - p0) * (1.0 - pe);

    s.basesB.resize(s.N);
    s.measB.resize(s.N);

    for (int i=0;i<s.N;i++) {
        char basis = (intrand(2)? 'X':'Z');
        s.basesB[i] = basis;

        bool ideal;
        char st = (i < (int)s.statesA.size()) ? s.statesA[i] : '0';
        if ((basis=='Z' && (st=='0' || st=='1'))) {
            ideal = (st=='1');    // Z: '0'→0, '1'→1
        } else if ((basis=='X' && (st=='+' || st=='-'))) {
            ideal = (st=='-');    // X: '+'→0, '−'→1
        } else {
            ideal = (intrand(2)!=0); // 异基随机
        }
        bool flip = (uniform(0,1) < p);
        bool meas = flip ? !ideal : ideal;
        s.measB[i] = (meas? '1':'0');
    }
}

void ApplicationLayerTest::computeConclusiveAndSample(Session& s) {
    std::vector<int> pool; pool.reserve(s.N);
    for (int i=0; i<s.N; ++i) {
        if (i < (int)s.conclusiveMask.size() && s.conclusiveMask[i]=='1')
            pool.push_back(i);
    }

    if ((int)pool.size() < 2) {
        sendAbort(s, s.bobId, "conclusive too short");
        s.phase = ABORT;
        return;
    }

    // 关键修改：按 conclusive 数量计算抽样数
    int M = std::max(1, (int)std::round(pool.size() * sampleFraction));

    std::string idxStr; idxStr.reserve(4*M);
    std::string bitsStr; bitsStr.reserve(M);

    // Fisher-Yates 随机选 M 个
    for (int k=0; k<M; ++k) {
        int r = intrand((int)pool.size() - k);
        int chosen = pool[r];
        std::swap(pool[r], pool[(int)pool.size()-1-k]);

        if (k) idxStr.push_back(',');
        idxStr += std::to_string(chosen);
        bitsStr.push_back(s.bitsA[chosen]);
    }

    s.sampleIdx   = idxStr;
    s.sampleBitsA = bitsStr;
    s.sampleCount = M;

    // 更新 S_conclusive
    s.S_conclusive = pool.size();
}


void ApplicationLayerTest::finalizeKeyAndColor(Session& s) {
    // SARG04：若有 conclusiveMask，则以可判定集合为准；否则沿用 BB84 的 siftMask
    int S = 0;
    bool useConclusive = useSARG04 && !s.conclusiveMask.empty();
    if (useConclusive) {
        for (char c: s.conclusiveMask) if (c=='1') ++S;
    } else {
        for (char c: s.siftMask) if (c=='1') ++S;
    }

    // SARG04 需扣除抽样位
    int S_eff;
    if (useConclusive) {
        S_eff = std::max(0, S - s.sampleCount);  // SARG04
    } else {
        S_eff = std::max(0, S - s.sampleCount);  // BB84 也要扣除
    }

    double rate = std::max(0.0, 1.0 - s.fEC*H2(s.qber));

    int safety;
    if (s.N < 100)       safety = 0;
    else if (s.N < 1000) safety = 16;
    else                 safety = 64;

    int L = std::max(0, (int)std::floor(S_eff * rate) - safety);



    if (L <= 0) {
        EV_WARN << "[APP] sid="<<s.sessionId<<" final L<=0, abort\n";
        s.phase = ABORT;
        return;
    }

    totalKeyBitsGenerated += L;

    s.finalKey.assign(L, '0');
    hasKey = true;
    peersWithKey.insert( ((int)myId==s.aliceId)? s.bobId : s.aliceId );

    // ===== MISSION-CRITICAL: inject key pool =====
    int peer = ((int)myId == s.aliceId) ? s.bobId : s.aliceId;

    keyPoolBits[peer] += L;
    keyConsumed[peer] += 0;

    EV_INFO << "[MISSION][KEY-POOL]"
            << " peer=" << peer
            << " add=" << L
            << " total=" << keyPoolBits[peer]
            << " t=" << simTime()
            << "\n";


    findHost()->getDisplayString().setTagArg("i", 1, "green");
    double keyRate = (s.N > 0) ? ((double)L / (double)s.N) : 0.0;

    int runId = getSimulation()->getEnvir()->getConfigEx()->getActiveRunNumber();

    EV_INFO << "[APP] DONE"
            << " sid=" << s.sessionId
            << " Nraw=" << s.N
            << " S=" << S
            << " S_eff=" << S_eff
            << " L=" << L
            << " qber=" << s.qber
            << " keyRate=" << keyRate
            << " protocol=" << (useConclusive ? "SARG04" : "BB84")
            << "\n";


}

void ApplicationLayerTest::sendAbortSid(int sid, int dst, const char* reason) {
    Session tmp; tmp.sessionId = sid; tmp.aliceId = (int)myId; tmp.bobId = dst; tmp.phase = ABORT;
    sendAbort(tmp, dst, reason);
}

void ApplicationLayerTest::cleanupSession(int sid) {
    auto it = sessions.find(sid);
    if (it == sessions.end()) return;

    int peer = ((int)myId == it->second.aliceId) ? it->second.bobId : it->second.aliceId;

    if (it->second.stepTimer) {
        cancelAndDelete(it->second.stepTimer);
        it->second.stepTimer = nullptr;
    }
    sessions.erase(it);

    auto itAct = activeWithPeer.find(peer);
    if (itAct != activeWithPeer.end() && itAct->second == sid) {
        activeWithPeer.erase(itAct);
    }
    cooldownUntil[peer] = simTime() + 1.0;
}


void ApplicationLayerTest::finish() {

    // ===== ONLY ONE NODE WRITES CSV =====
    int myId = getParentModule()->getIndex();  // node index

    if (myId != 0) {
        return;  // 其他节点直接退出，不写文件
    }

    // ===== Mission end time =====
    missionEndTime = simTime();

    double MMSR = (missionMsgRequired > 0)
        ? static_cast<double>(missionMsgSupported) / missionMsgRequired
        : 0.0;

    double MST = (missionStartTime >= SIMTIME_ZERO)
        ? (missionEndTime.dbl() - missionStartTime.dbl())
        : 0.0;

    int runId = getSimulation()->getEnvir()->getConfigEx()->getActiveRunNumber();

    std::ofstream csv("mission_qkd_results_new.csv", std::ios::app);

    if (csv.tellp() == 0) {
        csv << "runId,Nraw,L,MMSR,totalOutage,MST\n";
    }

    csv << runId << ","
        << Nraw << ","
        << totalKeyBitsGenerated << ","
        << MMSR << ","
        << missionOutageCount << ","
        << MST << "\n";

    csv.close();
}
