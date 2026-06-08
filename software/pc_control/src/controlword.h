#ifndef CONTROLWORD_H
#define CONTROLWORD_H

#include <cstdint>
#include <array>
#include <QString>
#include <QDebug>

// 96-bit control word mapped to 3× D-sub 37 connectors
// D-sub1: bits 0-31 (pins 6-37),  pins 1-5 = GND
// D-sub2: bits 32-63,              pins 1-5 = GND
// D-sub3: bits 64-95,              pins 1-5 = GND
//
// See docs/96位数据位映射定义.md for full mapping

class ControlWord {
public:
    ControlWord();

    // --- 天线控制 (Antenna V/H switches) ---
    // ant: 'A'..'H', pol: 'V' or 'H'
    void setAntenna(char antenna, char polarization, bool on);
    bool getAntenna(char antenna, char polarization) const;

    // --- 直通控制 (Direct-through, 仅 B, C, F, G 有效) ---
    void setDirectThrough(char antenna, bool enable);
    bool getDirectThrough(char antenna) const;

    // --- 参考通道 (Reference channel SP4T) ---
    void setReferenceChannel(int channel); // 1-4
    int getReferenceChannel() const;

    // --- IF 模块控制 (3 个模块 × 3 路 IF) ---
    // module: 1-3, ifch: 1-3 (IF1/IF2/IF3)
    void setIFBand(int module, int ifch, bool wide);
    bool getIFBand(int module, int ifch) const;

    void setIFAttenuation(int module, int ifch, int dB); // 0-15
    int getIFAttenuation(int module, int ifch) const;

    // --- 直接置位/清除任意位 ---
    void setBit(int bit, bool value);
    bool getBit(int bit) const;

    // --- CDC 收发 ---
    std::array<uint8_t, 12> toSendBytes() const;
    static ControlWord fromSendBytes(const std::array<uint8_t, 12> &bytes);

    // --- 模板合并 ---
    // 仅将 other 中被显式设置过的脏位合并到当前字，其他位保持不变
    void applyDirtyFrom(const ControlWord &other);
    void markAllDirty();
    const std::array<uint8_t, 12>& dirtyBytes() const { return m_dirtyMask; }
    void setRawBytes(const std::array<uint8_t, 12> &bytes,
                     const std::array<uint8_t, 12> &dirty);

    // --- 工具 ---
    void reset();
    QString toDebugString() const;
    void printBits() const;

    std::array<uint8_t, 12> rawBytes() const { return m_raw; }
    void setRawBytes(const std::array<uint8_t, 12> &bytes) { m_raw = bytes; m_dirtyMask.fill(0xFF); }

private:
    // Antenna bit lookup
    // Module 1 (bits 0-9): K1=A, K2=B, K3=C, K4=D
    // Module 2 (bits 10-19): K1=E, K2=F, K3=G, K4=H
    // J30J pin order: K3B, K3A, K4B, K4A, K1A, K1B, K2A, K2B, K2ZT, K3ZT
    int antennaBit(char antenna, char polarization) const;

    std::array<uint8_t, 12> m_raw;
    std::array<uint8_t, 12> m_dirtyMask{};
};

#endif // CONTROLWORD_H
