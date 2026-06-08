#include "controlword.h"
#include <QRegularExpression>

ControlWord::ControlWord() {
    m_raw.fill(0);
    m_dirtyMask.fill(0);
}

// --- Antenna bit mapping ---
// Module 1 (bits 0-9): antennas A,B,C,D mapped as K1=A, K2=B, K3=C, K4=D
// Module 2 (bits 10-19): antennas E,F,G,H mapped as K1=E, K2=F, K3=G, K4=H
//
// J30J pin order within each module: K3B, K3A, K4B, K4A, K1A, K1B, K2A, K2B, K2ZT, K3ZT
// KxA=V, KxB=H
//
// Bit layout per module:
//   0(+10m): K3B = antenna(module, K3) V
//   1(+10m): K3A = antenna(module, K3) H
//   2(+10m): K4B = antenna(module, K4) V
//   3(+10m): K4A = antenna(module, K4) H
//   4(+10m): K1A = antenna(module, K1) V
//   5(+10m): K1B = antenna(module, K1) H
//   6(+10m): K2A = antenna(module, K2) V
//   7(+10m): K2B = antenna(module, K2) H
//   8(+10m): K2ZT = antenna(module, K2) direct-through
//   9(+10m): K3ZT = antenna(module, K3) direct-through

int ControlWord::antennaBit(char antenna, char polarization) const {
    // Module assignment
    // Module 1: A→K1, B→K2, C→K3, D→K4
    // Module 2: E→K1, F→K2, G→K3, H→K4
    struct { char ant; int baseBit; int kPos; } map;
    switch (antenna) {
    case 'A': map = {'A', 0, 1}; break; // K1, base 0
    case 'B': map = {'B', 0, 2}; break; // K2, base 0
    case 'C': map = {'C', 0, 3}; break; // K3, base 0
    case 'D': map = {'D', 0, 4}; break; // K4, base 0
    case 'E': map = {'E', 10, 1}; break; // K1, base 10
    case 'F': map = {'F', 10, 2}; break; // K2, base 10
    case 'G': map = {'G', 10, 3}; break; // K3, base 10
    case 'H': map = {'H', 10, 4}; break; // K4, base 10
    default: return -1;
    }

    bool isV = (polarization == 'V' || polarization == 'v');
    // J30J pin order: K3(0-1), K4(2-3), K1(4-5), K2(6-7), K2ZT(8), K3ZT(9)
    // KxA=V (even offset), KxB=H (odd offset)
    static const int kToBit[5] = {0, 4, 6, 0, 2}; // K1→4, K2→6, K3→0, K4→2
    int bitOff = kToBit[map.kPos] + (isV ? 0 : 1);
    return map.baseBit + bitOff;
}

void ControlWord::setAntenna(char antenna, char polarization, bool on) {
    int bit = antennaBit(antenna, polarization);
    if (bit >= 0) setBit(bit, on);
}

bool ControlWord::getAntenna(char antenna, char polarization) const {
    int bit = antennaBit(antenna, polarization);
    return (bit >= 0) && getBit(bit);
}

// Direct-through: K2ZT for K2 antennas (B, F), K3ZT for K3 antennas (C, G)
void ControlWord::setDirectThrough(char antenna, bool enable) {
    int bit = -1;
    switch (antenna) {
    case 'B': bit = 8; break;  // K2ZT, module 1
    case 'C': bit = 9; break;  // K3ZT, module 1
    case 'F': bit = 18; break; // K2ZT, module 2
    case 'G': bit = 19; break; // K3ZT, module 2
    }
    if (bit >= 0) setBit(bit, enable);
}

bool ControlWord::getDirectThrough(char antenna) const {
    switch (antenna) {
    case 'B': return getBit(8);
    case 'C': return getBit(9);
    case 'F': return getBit(18);
    case 'G': return getBit(19);
    }
    return false;
}

// --- Reference channel: bits 20-21 ---
void ControlWord::setReferenceChannel(int channel) {
    switch (channel) {
    case 1: setBit(20, true);  setBit(21, true);  break;
    case 2: setBit(20, true);  setBit(21, false); break;
    case 3: setBit(20, false); setBit(21, true);  break;
    case 4: setBit(20, false); setBit(21, false); break;
    }
}

int ControlWord::getReferenceChannel() const {
    bool a = getBit(20), b = getBit(21);
    if (a && b) return 1;
    if (a && !b) return 2;
    if (!a && b) return 3;
    return 4;
}

// --- IF modules (25中频模块 ×3) ---
// D-sub2:  C1(位32-49, 18位)+C2(位50-63, 14位)
// D-sub3:  C2续(位64-67, 4位)+C3(位68-85, 18位)+备用(86-95)
//
// Per module bit layout (offset 0, base见 ifModuleBase):
//   off+0: KIF11 (IF1窄带入)
//   off+1: KIF12 (IF1窄带出)
//   off+2: KIF21 (IF2窄带入)
//   off+3: KIF22 (IF2窄带出)
//   off+4: KIF31 (IF3窄带入)
//   off+5: KIF32 (IF3窄带出)
//   off+6..9: 1ATT1/2/4/8 (IF1衰减, 4位连续)
//   off+10..13: 2ATT1/2/4/8 (IF2衰减, 4位)
//   off+14..17: 3ATT1/2/4/8 (IF3衰减, 4位)

static int ifModuleBase(int module) {
    switch (module) {
    case 1: return 32;
    case 2: return 50;
    case 3: return 68;
    }
    return -1;
}

void ControlWord::setIFBand(int module, int ifch, bool wide) {
    int base = ifModuleBase(module);
    if (base < 0 || ifch < 1 || ifch > 3) return;
    // KIFx1 (in) = wide, KIFx2 (out) = !wide
    // KIF11/12 at off+0/1, KIF21/22 at off+2/3, KIF31/32 at off+4/5
    int inBit = base + (ifch - 1) * 2;      // KIF11,21,31
    int outBit = base + (ifch - 1) * 2 + 1; // KIF12,22,32
    setBit(inBit, wide);
    setBit(outBit, !wide);
}

bool ControlWord::getIFBand(int module, int ifch) const {
    int base = ifModuleBase(module);
    if (base < 0 || ifch < 1 || ifch > 3) return false;
    int inBit = base + (ifch - 1) * 2;
    return getBit(inBit);
}

void ControlWord::setIFAttenuation(int module, int ifch, int dB) {
    int base = ifModuleBase(module);
    if (base < 0 || ifch < 1 || ifch > 3) return;
    dB = qBound(0, dB, 15);
    int attBase = base + 6 + (ifch - 1) * 4;
    for (int i = 0; i < 4; i++)
        setBit(attBase + i, (dB >> i) & 1);
}

int ControlWord::getIFAttenuation(int module, int ifch) const {
    int base = ifModuleBase(module);
    if (base < 0 || ifch < 1 || ifch > 3) return 0;
    int attBase = base + 6 + (ifch - 1) * 4;
    int val = 0;
    for (int i = 0; i < 4; i++)
        if (getBit(attBase + i)) val |= (1 << i);
    return val;
}

void ControlWord::setBit(int bit, bool value) {
    if (bit < 0 || bit > 95) return;
    int byteIdx = bit / 8;
    int bitIdx = 7 - (bit % 8); // SPI MSB first: bit 0 → MSB of byte
    uint8_t mask = 1u << bitIdx;
    if (value)
        m_raw[byteIdx] |= mask;
    else
        m_raw[byteIdx] &= ~mask;
    m_dirtyMask[byteIdx] |= mask;
}

bool ControlWord::getBit(int bit) const {
    if (bit < 0 || bit > 95) return false;
    int bitIdx = 7 - (bit % 8);
    return (m_raw[bit / 8] >> bitIdx) & 1;
}

std::array<uint8_t, 12> ControlWord::toSendBytes() const {
    return m_raw;
}

ControlWord ControlWord::fromSendBytes(const std::array<uint8_t, 12> &bytes) {
    ControlWord cw;
    cw.m_raw = bytes;
    return cw;
}

void ControlWord::reset() {
    m_raw.fill(0);
    m_dirtyMask.fill(0);
}

QString ControlWord::toDebugString() const {
    QString s;
    for (int i = 0; i < 12; i++) {
        s += QString("%1 ").arg(m_raw[i], 2, 16, QChar('0'));
    }
    return s;
}

void ControlWord::printBits() const {
    QString s;
    for (int i = 95; i >= 0; i--) {
        s += (getBit(i) ? '1' : '0');
        if (i % 8 == 0 && i > 0) s += ' ';
    }
    qDebug().noquote() << s;
}

void ControlWord::applyDirtyFrom(const ControlWord &other) {
    for (int i = 0; i < 12; i++)
        m_raw[i] = (m_raw[i] & ~other.m_dirtyMask[i]) | (other.m_raw[i] & other.m_dirtyMask[i]);
}

void ControlWord::markAllDirty() {
    m_dirtyMask.fill(0xFF);
}

void ControlWord::setRawBytes(const std::array<uint8_t, 12> &bytes,
                               const std::array<uint8_t, 12> &dirty) {
    m_raw = bytes;
    m_dirtyMask = dirty;
}
