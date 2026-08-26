#ifndef A2LMODEL_H
#define A2LMODEL_H

#include <QString>
#include <QByteArray>
#include <QVector>
#include <QtGlobal>

// Minimal ASAP2/A2L reader: extracts the CHARACTERISTIC list (calibratable
// objects) from an A2L file so the GUI's calibration table can be built from
// the firmware description instead of a hardcoded list. Only the record
// layouts this project uses are understood (float32 / uint32 / uint8).

enum class A2lType { Float32, Uint32, Uint16, Uint8 };

inline int a2lTypeSize(A2lType t)
{
    switch (t) {
    case A2lType::Float32: return 4;
    case A2lType::Uint32:  return 4;
    case A2lType::Uint16:  return 2;
    case A2lType::Uint8:   return 1;
    }
    return 4;
}

struct A2lChar {
    QString name;          // CHARACTERISTIC identifier (e.g. GPIO_P00_00_enable)
    QString desc;          // quoted description that follows the identifier
    QString unit;          // PHYS_UNIT, empty if none
    quint32 addr = 0;      // ECU address from the VALUE line
    A2lType type = A2lType::Float32;
    double  lo   = 0.0;    // lower limit
    double  hi   = 0.0;    // upper limit
};

// One MEASUREMENT: a read-only signal in the firmware's measurement block.
// Unlike A2lChar these carry no record layout - the base type sits directly in
// the MEASUREMENT header - and they are grouped for display by name prefix.
struct A2lMeas {
    QString name;          // MEASUREMENT identifier (e.g. AhrsRoll)
    QString desc;          // quoted description that follows the identifier
    QString unit;          // PHYS_UNIT, empty if none
    quint32 addr = 0;      // ECU_ADDRESS
    A2lType type = A2lType::Float32;
    double  lo   = 0.0;
    double  hi   = 0.0;
    bool    isBitMask = false;  // BIT_MASK view (a single diagnostics bit)
    quint32 bitMask   = 0;      // the mask itself, when isBitMask
};

class A2lModel {
public:
    // Parse every CHARACTERISTIC in the file. Returns the list (empty on
    // failure); *err, if given, holds a human-readable reason on failure.
    static QVector<A2lChar> parseFile(const QString &path, QString *err = nullptr);

    // Parse every MEASUREMENT in the file. Lets the Sensors tab be built from
    // the firmware description instead of a hardcoded signal list: adding a
    // MEASUREMENT to the A2L is enough to make it appear in the GUI.
    static QVector<A2lMeas> parseMeasurements(const QString &path, QString *err = nullptr);

    // Decode one measurement out of a raw copy of the firmware's measurement
    // block. blockBase is the address the raw buffer starts at, so the offset
    // is (m.addr - blockBase). Returns false if the buffer does not cover it.
    static bool decode(const QByteArray &raw, quint32 blockBase,
                       const A2lMeas &m, double *out);

    // Number of bytes that must be fetched from blockBase for every MEASUREMENT
    // in the list to be decodable: max(addr - base + sizeof(type)).
    //
    // This is what lets the transport size itself from the firmware
    // description instead of a hardcoded block length. Add a MEASUREMENT to the
    // A2L and the fetched block grows to cover it -- no GUI constant to bump,
    // and no silently dropped signal when someone forgets.
    //
    // Measurements below blockBase, or beyond limit bytes past it, are ignored:
    // Xcp_Cal / Xcp_Nvm / Xcp_Gpio sit 256 bytes apart, so a stray object in
    // another block must not drag the fetch length across the gap.
    static int blockExtent(const QVector<A2lMeas> &meas, quint32 blockBase,
                           int limit = 256);

    // One contiguous region of firmware memory that has to be fetched.
    struct BlockRange {
        quint32 base = 0;
        int     size = 0;
    };

    // Every block the MEASUREMENTs live in, derived from their addresses.
    //
    // The firmware pins its blocks 256 bytes apart (Xcp_Data 0x70030000,
    // Xcp_Fusion 0x70030500, ...), so grouping by the address with the low byte
    // masked off recovers that layout without the GUI having to be told it.
    //
    // This exists because a hardcoded single base is a bug that recurs. The
    // navigation state was added in its own block and every one of its signals
    // was invisible here -- present in the A2L, parsed, listed, and never
    // fetched, because the transport only ever asked for 0x70030000. Deriving
    // the set means the next block appears on its own.
    static QVector<BlockRange> blockRanges(const QVector<A2lMeas> &meas);

    // Decode against whichever fetched block actually contains m.
    //
    // Takes the bases and buffers straight off XcpClient::Measurements, so a
    // caller never has to know which block a signal lives in -- which is the
    // whole point: the Sensors tab and the plot picker list signals from the
    // A2L, and any of them may be in any block.
    static bool decodeFrom(const QVector<quint32> &bases,
                           const QVector<QByteArray> &raws,
                           const A2lMeas &m, double *out);
};

#endif // A2LMODEL_H
