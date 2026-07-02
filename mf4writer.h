#ifndef MF4WRITER_H
#define MF4WRITER_H

#include <QString>
#include <QVector>

// Minimal ASAM MDF 4.10 writer: one data group / one channel group with
// fixed-length records. Samples are buffered in RAM while logging and the
// complete file is written on save() - at 10 Hz this stays tiny.
class Mf4Writer
{
public:
    struct Sample {
        double  t     = 0.0;    // seconds since logging start (master channel)
        float   dts   = 0.0f;   // die temperature PMS DTS [degC]
        float   dtsc  = 0.0f;   // die temperature SCU DTSC [degC]
        float   vdd   = 0.0f;   // [V]
        float   vddp3 = 0.0f;   // [V]
        float   vext  = 0.0f;   // [V]
        quint32 tick  = 0;      // board uptime [ms]
        quint32 diag  = 0;      // diagnostics bitmask
    };

    void clear()                      { m_samples.clear(); }
    void append(const Sample &s)      { m_samples.append(s); }
    int  count() const                { return int(m_samples.size()); }

    // Writes the buffered samples as a valid MDF 4.10 file.
    bool save(const QString &path, QString *errorOut = nullptr) const;

private:
    QVector<Sample> m_samples;
};

#endif // MF4WRITER_H
