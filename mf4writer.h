#ifndef MF4WRITER_H
#define MF4WRITER_H

#include <QString>
#include <QVector>

// Minimal ASAM MDF 4.10 writer: one data group / one channel group with
// fixed-length records. The channel set is chosen at run time (the user picks
// which measurements to log), so records carry an implicit "time" master
// channel followed by the selected data channels. Samples are buffered in RAM
// while logging and the complete file is written on save() - at 10 Hz this
// stays tiny.
class Mf4Writer
{
public:
    struct Channel {
        QString name;
        QString unit;              // empty = no unit
        bool    isFloat = true;    // true = float32, false = uint32
    };

    // Configure the channel set (excluding the implicit time master) and drop
    // any buffered samples. Call once before a logging session.
    void begin(const QVector<Channel> &channels);
    void clear();                                       // drop samples only
    void append(double t, const QVector<double> &values); // values.size()==channels
    int  count() const { return int(m_time.size()); }

    // Writes the buffered samples as a valid MDF 4.10 file.
    bool save(const QString &path, QString *errorOut = nullptr) const;

private:
    QVector<Channel>         m_channels;
    QVector<double>          m_time;
    QVector<QVector<double>> m_rows;    // parallel to m_time, one value per channel
};

#endif // MF4WRITER_H
