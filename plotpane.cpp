#include "plotpane.h"
#include "plotwidget.h"

#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDialog>
#include <QDialogButtonBox>
#include <QCheckBox>
#include <QScrollArea>
#include <QLineEdit>
#include <QGroupBox>
#include <QSet>

namespace {
// Address of the firmware measurement block; A2L ECU_ADDRESSes are absolute.
constexpr quint32 XCP_DATA_ADDR = 0x70030000u;

// Group heading for a signal, from its name prefix - same convention as the
// Sensors tab, so the picker reads the same way.
QString groupTitleFor(const QString &name)
{
    if (name.startsWith("Baro")) return QStringLiteral("Barometer");
    if (name.startsWith("Imu"))  return QStringLiteral("IMU");
    if (name.startsWith("Mag"))  return QStringLiteral("Magnetometer");
    if (name.startsWith("Gnss")) return QStringLiteral("GNSS");
    if (name.startsWith("Core")) return QStringLiteral("Core load");
    if (name.startsWith("Eth"))  return QStringLiteral("Ethernet");
    return QStringLiteral("Onboard");
}

const char *kGroupOrder[] = { "Onboard", "Barometer", "IMU", "Magnetometer",
                              "GNSS", "Core load", "Ethernet" };
} // namespace

PlotPane::PlotPane(int index, const QVector<A2lMeas> &available, QWidget *parent)
    : QWidget(parent), m_available(available), m_index(index)
{
    m_plot  = new PlotWidget;
    m_title = new QLabel(tr("Plot %1 - no signals").arg(index + 1));

    auto *edit = new QPushButton(tr("Signals..."));
    edit->setToolTip(tr("Choose which measurements this plot draws"));
    connect(edit, &QPushButton::clicked, this, &PlotPane::editSignals);

    auto *close = new QPushButton(tr("Remove"));
    connect(close, &QPushButton::clicked, this, [this]() { emit removeRequested(this); });

    auto *head = new QHBoxLayout;
    head->addWidget(m_title, 1);
    head->addWidget(edit);
    head->addWidget(close);

    auto *col = new QVBoxLayout(this);
    col->setContentsMargins(4, 4, 4, 4);
    col->addLayout(head);
    col->addWidget(m_plot, 1);

    setMinimumHeight(240);
}

void PlotPane::setAvailable(const QVector<A2lMeas> &available)
{
    m_available = available;

    // Keep the selection, minus anything the new A2L no longer describes.
    const QVector<QString> keep = selectedNames();
    setSelectedNames(keep);
}

QVector<QString> PlotPane::selectedNames() const
{
    QVector<QString> names;
    names.reserve(m_selected.size());
    for (const A2lMeas &m : m_selected)
        names.append(m.name);
    return names;
}

void PlotPane::setSelectedNames(const QVector<QString> &names)
{
    const QSet<QString> want(names.begin(), names.end());
    m_selected.clear();
    for (const A2lMeas &m : m_available)
        if (want.contains(m.name))
            m_selected.append(m);
    rebuildSeries();
    emit configChanged();
}

void PlotPane::rebuildSeries()
{
    m_plot->clearSeries();
    m_series.clear();

    for (int i = 0; i < m_selected.size(); ++i) {
        const A2lMeas &m = m_selected[i];
        const QString label = m.unit.isEmpty()
                                  ? m.name
                                  : QString("%1 [%2]").arg(m.name, m.unit);
        // Evenly spaced hues so adjacent traces stay distinguishable.
        const QColor color = QColor::fromHsv((i * 67) % 360, 200, 200);
        const int sid = m_plot->addSeries(label, color);
        m_plot->setSeriesVisible(sid, true);
        m_series.append(sid);
    }

    m_title->setText(m_selected.isEmpty()
                         ? tr("Plot %1 - no signals").arg(m_index + 1)
                         : tr("Plot %1 - %2 signal(s)").arg(m_index + 1)
                               .arg(m_selected.size()));
}

void PlotPane::append(double t, const QByteArray &raw, bool baroPresent, bool imuPresent)
{
    if (raw.isEmpty())
        return;

    for (int i = 0; i < m_selected.size() && i < m_series.size(); ++i) {
        const A2lMeas &m = m_selected[i];
        // A missing sensor reports zeros; plotting them would look like data.
        if (!baroPresent && m.name.startsWith("Baro"))
            continue;
        if (!imuPresent && m.name.startsWith("Imu"))
            continue;

        double v = 0.0;
        if (A2lModel::decode(raw, XCP_DATA_ADDR, m, &v))
            m_plot->appendPoint(m_series[i], t, v);
    }
}

void PlotPane::clearData()
{
    m_plot->clearData();
}

void PlotPane::editSignals()
{
    QDialog dlg(this);
    dlg.setWindowTitle(tr("Signals for plot %1").arg(m_index + 1));
    auto *v = new QVBoxLayout(&dlg);

    auto *filter = new QLineEdit;
    filter->setPlaceholderText(tr("Filter by name..."));
    v->addWidget(filter);

    auto *host = new QWidget;
    auto *col  = new QVBoxLayout(host);

    // Keep the vector alive in a local. Calling selectedNames() twice inline
    // would build the set from iterators into two separate temporaries, both
    // already destroyed - which crashed as soon as the selection was non-empty.
    const QVector<QString> current = selectedNames();
    const QSet<QString> chosen(current.begin(), current.end());
    QVector<QCheckBox *> boxes;
    QVector<int>         boxToMeas;

    // Grouped exactly like the Sensors tab so the two read alike.
    for (const char *g : kGroupOrder) {
        QVector<int> members;
        for (int i = 0; i < m_available.size(); ++i)
            if (!m_available[i].isBitMask &&
                groupTitleFor(m_available[i].name) == QLatin1String(g))
                members.append(i);
        if (members.isEmpty())
            continue;

        auto *box    = new QGroupBox(QString::fromLatin1(g));
        auto *boxCol = new QVBoxLayout(box);
        for (int i : members) {
            const A2lMeas &m = m_available[i];
            auto *cb = new QCheckBox(m.unit.isEmpty()
                                         ? m.name
                                         : QString("%1 [%2]").arg(m.name, m.unit));
            cb->setToolTip(m.desc);
            cb->setChecked(chosen.contains(m.name));
            boxCol->addWidget(cb);
            boxes.append(cb);
            boxToMeas.append(i);
        }
        col->addWidget(box);
    }
    col->addStretch(1);

    auto *scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setWidget(host);
    scroll->setMinimumSize(420, 460);
    v->addWidget(scroll, 1);

    connect(filter, &QLineEdit::textChanged, &dlg, [&boxes](const QString &t) {
        for (QCheckBox *b : boxes)
            b->setVisible(t.isEmpty() || b->text().contains(t, Qt::CaseInsensitive));
    });

    auto *clearBtn = new QPushButton(tr("Clear all"));
    connect(clearBtn, &QPushButton::clicked, &dlg, [&boxes]() {
        for (QCheckBox *b : boxes) b->setChecked(false);
    });

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto *row = new QHBoxLayout;
    row->addWidget(clearBtn);
    row->addStretch(1);
    row->addWidget(buttons);
    v->addLayout(row);

    if (dlg.exec() != QDialog::Accepted)
        return;

    QVector<QString> picked;
    for (int i = 0; i < boxes.size(); ++i)
        if (boxes[i]->isChecked())
            picked.append(m_available[boxToMeas[i]].name);
    setSelectedNames(picked);
}
