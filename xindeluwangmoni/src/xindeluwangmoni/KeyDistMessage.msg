import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;
import veins.base.utils.SimpleAddress;
import veins.modules.application.traci.TraCIDemo11pMessage;

namespace xindeluwangmoni;

packet KeyDistMessage extends veins::BaseFrame1609_4{
    int    targetId;   // 目标车辆的 myId
    string key;        // 固定为 "111"
}