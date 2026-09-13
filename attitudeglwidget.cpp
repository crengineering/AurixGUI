#include "attitudeglwidget.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QShowEvent>
#include <QHideEvent>
#include <QPainter>
#include <QFont>
#include <QPalette>
#include <QtMath>
#include <QDebug>
#include <cmath>

namespace {

// attribute/varying (GLSL ES 1.00 style) rather than in/out core-profile
// syntax: QOpenGLWidget on this project's Windows/MinGW build hands out
// whatever context the platform gives it (desktop compatibility or ANGLE
// GLES), and this is the subset both accept without requesting a specific
// core profile that could fail to initialise on the bench machine's driver
// (SWE1-GUI-004).
const char *kVertexShader = R"(
attribute vec3 aPos;
attribute vec3 aColor;
uniform mat4 uMvp;
varying vec3 vColor;
void main() {
    gl_Position = uMvp * vec4(aPos, 1.0);
    vColor = aColor;
}
)";

const char *kFragmentShader = R"(
#ifdef GL_ES
precision mediump float;
#endif
varying vec3 vColor;
uniform float uLive;
// The ground grid's colour is derived from the current QPalette (light vs.
// dark OS theme) and supplied here as a uniform, recomputed fresh every
// frame from AttitudeGLWidget::palette() -- not baked into its VBO at
// buildGeometry() time, which would otherwise go stale until the widget was
// recreated after a live theme switch (SWE1-GUI-002 review fix).
uniform float uUseOverrideColor;
uniform vec3  uOverrideColor;
void main() {
    vec3 base = mix(vColor, uOverrideColor, uUseOverrideColor);
    // Stale/frozen pose is shown desaturated, never as if it were live --
    // the same "never present a stale number as live" rule applied to the
    // 3D pose itself (SWE1-GUI-003).
    float gray = dot(base, vec3(0.299, 0.587, 0.114));
    vec3 c = mix(vec3(gray), base, uLive);
    gl_FragColor = vec4(c, 1.0);
}
)";

AttitudeVertex vertexOf(const QVector3D &p, const QVector3D &c)
{
    return { p.x(), p.y(), p.z(), c.x(), c.y(), c.z() };
}

// A box whose long/wide/high extents run along arbitrary orthogonal axes, not
// just the world axes -- what makes the four diagonal arms as easy to build
// as the axis-aligned body. Winding is not guaranteed consistent from every
// face, which is fine: GL_CULL_FACE is deliberately never enabled here, so a
// backwards face is still drawn (depth testing alone gives the right result
// for solid low-poly shapes like these).
void addOrientedBox(QVector<AttitudeVertex> &out, const QVector3D &center,
                    const QVector3D &axisLong, float halfLong,
                    const QVector3D &axisWide, float halfWide,
                    const QVector3D &axisHigh, float halfHigh,
                    const QVector3D &color)
{
    const QVector3D L = axisLong.normalized() * halfLong;
    const QVector3D W = axisWide.normalized() * halfWide;
    const QVector3D H = axisHigh.normalized() * halfHigh;

    const QVector3D c[8] = {
        center - L - W - H, center + L - W - H, center + L + W - H, center - L + W - H,
        center - L - W + H, center + L - W + H, center + L + W + H, center - L + W + H,
    };

    auto quad = [&](int a, int b, int cc, int d) {
        const int idx[6] = { a, b, cc, a, cc, d };
        for (int i = 0; i < 6; ++i)
            out.append(vertexOf(c[idx[i]], color));
    };

    quad(0, 1, 2, 3);
    quad(7, 6, 5, 4);
    quad(4, 5, 1, 0);
    quad(3, 2, 6, 7);
    quad(0, 3, 7, 4);
    quad(1, 5, 6, 2);
}

// Flat disc (fan) in the local XY plane -- the four rotor discs, lying
// horizontal in the body frame.
void addDisc(QVector<AttitudeVertex> &out, const QVector3D &center, float radius,
            int segs, const QVector3D &color)
{
    for (int i = 0; i < segs; ++i) {
        const float a0 = 2.0f * float(M_PI) * float(i) / float(segs);
        const float a1 = 2.0f * float(M_PI) * float(i + 1) / float(segs);
        const QVector3D p0 = center + QVector3D(radius * std::cos(a0), radius * std::sin(a0), 0.0f);
        const QVector3D p1 = center + QVector3D(radius * std::cos(a1), radius * std::sin(a1), 0.0f);
        out.append(vertexOf(center, color));
        out.append(vertexOf(p0, color));
        out.append(vertexOf(p1, color));
    }
}

// Small pyramid (apex + base fan) used as the nose marker: unmistakable,
// distinctly coloured, points forward off the body's front face.
void addPyramid(QVector<AttitudeVertex> &out, const QVector3D &apex, const QVector3D &baseCenter,
                float radius, int segs, const QVector3D &color)
{
    const QVector3D dir = (apex - baseCenter).normalized();
    const QVector3D arbitrary = std::fabs(QVector3D::dotProduct(dir, QVector3D(0, 0, 1))) < 0.9f
                                     ? QVector3D(0, 0, 1) : QVector3D(0, 1, 0);
    const QVector3D u = QVector3D::crossProduct(dir, arbitrary).normalized();
    const QVector3D v = QVector3D::crossProduct(dir, u).normalized();

    for (int i = 0; i < segs; ++i) {
        const float a0 = 2.0f * float(M_PI) * float(i) / float(segs);
        const float a1 = 2.0f * float(M_PI) * float(i + 1) / float(segs);
        const QVector3D p0 = baseCenter + radius * (std::cos(a0) * u + std::sin(a0) * v);
        const QVector3D p1 = baseCenter + radius * (std::cos(a1) * u + std::sin(a1) * v);
        out.append(vertexOf(apex, color));
        out.append(vertexOf(p0, color));
        out.append(vertexOf(p1, color));
        out.append(vertexOf(baseCenter, color));
        out.append(vertexOf(p1, color));
        out.append(vertexOf(p0, color));
    }
}

} // namespace

QMatrix4x4 nedToGlBasis()
{
    // Row-major. GL.x = E, GL.y = -D, GL.z = -N: N points into the room (away
    // from the default camera, which looks down -Z), E to the right, D down
    // maps to -Y (down on screen). NED is right-handed (N x E = D); this
    // basis is too (E x -D = -N), so the mapping is a pure rotation, never a
    // mirror -- attitudeSelfCheck() below proves it numerically for the three
    // canonical cases that matter (nose-up, right-side-down, yaw+90).
    return QMatrix4x4(
        0.0f, 1.0f,  0.0f, 0.0f,
        0.0f, 0.0f, -1.0f, 0.0f,
       -1.0f, 0.0f,  0.0f, 0.0f,
        0.0f, 0.0f,  0.0f, 1.0f);
}

QMatrix4x4 attitudeModelMatrix(const QQuaternion &bodyToNed)
{
    QMatrix4x4 rot;
    rot.rotate(bodyToNed);
    return nedToGlBasis() * rot;
}

bool attitudeSelfCheck(QString *report)
{
    struct Case {
        const char *name;
        QVector3D   axis;
        float       angleDeg;
        QVector3D   bodyPoint;   // the body-frame point/direction we track
        QVector3D   expectedGl; // where it must end up on screen
    };
    // Angles match the firmware's own bench-check table (Ahrs.h lines 30-34):
    // nose up -> pitch ~+90, right side down -> roll ~+90, level turned 90
    // -> yaw advances by 90. Tracking the forward axis (nose) and the right
    // axis (a right-side marker) makes "rises"/"drops" a simple sign check on
    // the resulting screen-space Y.
    const Case cases[] = {
        { "nose up +90 (pitch)",        QVector3D(0, 1, 0), 90.0f, QVector3D(1, 0, 0), QVector3D(0,  1, 0) },
        { "right side down +90 (roll)", QVector3D(1, 0, 0), 90.0f, QVector3D(0, 1, 0), QVector3D(0, -1, 0) },
        { "yaw +90 (heading turn)",     QVector3D(0, 0, 1), 90.0f, QVector3D(1, 0, 0), QVector3D(1,  0, 0) },
    };

    const QMatrix4x4 conv = nedToGlBasis();
    bool allOk = true;
    QString out;
    for (const Case &c : cases) {
        const QQuaternion q = QQuaternion::fromAxisAndAngle(c.axis, c.angleDeg);
        const QVector3D bodyInNed = q.rotatedVector(c.bodyPoint);
        const QVector4D glV = conv * QVector4D(bodyInNed, 0.0f);   // direction: w=0
        const QVector3D gl(glV.x(), glV.y(), glV.z());
        const bool ok = (gl - c.expectedGl).lengthSquared() < 1e-3f;
        allOk = allOk && ok;
        out += QString("  %1: expected GL(%2,%3,%4) observed GL(%5,%6,%7) -- %8\n")
                   .arg(c.name)
                   .arg(double(c.expectedGl.x()), 0, 'f', 2).arg(double(c.expectedGl.y()), 0, 'f', 2).arg(double(c.expectedGl.z()), 0, 'f', 2)
                   .arg(double(gl.x()), 0, 'f', 2).arg(double(gl.y()), 0, 'f', 2).arg(double(gl.z()), 0, 'f', 2)
                   .arg(ok ? "PASS" : "FAIL");
    }
    if (report)
        *report = out;
    return allOk;
}

AttitudeGLWidget::AttitudeGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(280, 220);
    setToolTip(tr("Drag: orbit | Wheel: zoom"));

    m_renderTimer.setInterval(16);   // ~60 Hz display rate (SYS2-GUI-002)
    connect(&m_renderTimer, &QTimer::timeout, this, QOverload<>::of(&QWidget::update));
    // Not started here: updateTimerState() only runs it while live and
    // visible (blocker 7) -- nothing to interpolate towards yet anyway.

    // Unconditional in every build configuration, not just debug: build.bat
    // sets no CMAKE_BUILD_TYPE, so Qt compiles with -DQT_NO_DEBUG and
    // Q_ASSERT is a no-op in the exact binary that ships. The result is
    // cached in m_selfCheckOk and read by AttitudeView, which shows a visible
    // failure banner and refuses to go live -- qDebug alone is invisible on
    // a WIN32_EXECUTABLE with no console. attitude_selfcheck (see
    // tools/attitude_selfcheck_main.cpp) is the automated, headless evidence
    // for this check (SWE1-GUI-001, vv: unit).
    QString report;
    m_selfCheckOk = attitudeSelfCheck(&report);
    qDebug().noquote() << "[Attitude] handedness self-check"
                       << (m_selfCheckOk ? "PASS" : "FAIL") << "\n" << report;
}

AttitudeGLWidget::~AttitudeGLWidget()
{
    makeCurrent();
    m_modelVbo.destroy();
    m_gridVbo.destroy();
    m_triadVbo.destroy();
    doneCurrent();
}

void AttitudeGLWidget::pushQuaternion(float w, float x, float y, float z)
{
    if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        return;   // NaN-guard: never let a broken frame reach the renderer

    QQuaternion q(w, x, y, z);
    const float n = q.length();
    if (n < 0.5f || n > 1.5f)
        return;   // far from unit norm: not a valid attitude sample, ignore
    q.normalize();

    if (m_haveSample) {
        const qint64 dt = m_sinceSample.restart();
        if (dt > 20 && dt < 2000)          // ignore startup/stall outliers
            m_sampleIntervalMs = dt;
        // Continue the SLERP from wherever the display currently is, not from
        // the old target -- avoids a visible snap if a repaint was late.
        m_prevQuat = m_displayQuat;
    } else {
        m_prevQuat = q;
        m_sinceSample.start();
    }
    m_latestQuat = q;
    m_haveSample = true;
}

void AttitudeGLWidget::setLive(bool live)
{
    if (m_live == live)
        return;
    m_live = live;
    updateTimerState();
    // One immediate repaint so the live<->frozen transition (colour <->
    // desaturated) shows right away instead of waiting for the next tick --
    // relevant in particular when going live->frozen, since the timer is
    // about to stop.
    update();
}

void AttitudeGLWidget::updateTimerState()
{
    // The ~60 Hz repaint only ever runs while there is something to
    // interpolate towards and somewhere to show it -- not a free-running
    // timer that keeps ticking in a hidden or frozen tab (blocker 7).
    const bool shouldRun = m_live && isVisible();
    if (shouldRun && !m_renderTimer.isActive())
        m_renderTimer.start();
    else if (!shouldRun && m_renderTimer.isActive())
        m_renderTimer.stop();
}

void AttitudeGLWidget::showEvent(QShowEvent *e)
{
    QOpenGLWidget::showEvent(e);
    updateTimerState();
    update();
}

void AttitudeGLWidget::hideEvent(QHideEvent *e)
{
    QOpenGLWidget::hideEvent(e);
    updateTimerState();
}

void AttitudeGLWidget::resetView()
{
    m_camYawDeg   = kDefaultYawDeg;
    m_camPitchDeg = kDefaultPitchDeg;
    m_camDist     = kDefaultDist;
    update();
}

void AttitudeGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);

    const QColor bg = palette().color(QPalette::Base);
    glClearColor(float(bg.redF()), float(bg.greenF()), float(bg.blueF()), 1.0f);

    m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader);
    m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader);
    m_program.bindAttributeLocation("aPos", 0);
    m_program.bindAttributeLocation("aColor", 1);
    m_shaderOk = m_program.link();
    if (!m_shaderOk)
        qWarning() << "[Attitude] shader link failed:" << m_program.log();

    buildGeometry();
}

void AttitudeGLWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    m_proj.setToIdentity();
    m_proj.perspective(45.0f, h > 0 ? float(w) / float(h) : 1.0f, 0.05f, 50.0f);
}

QMatrix4x4 AttitudeGLWidget::camera() const
{
    const float yaw   = qDegreesToRadians(m_camYawDeg);
    const float pitch = qDegreesToRadians(m_camPitchDeg);
    const QVector3D eye(m_camDist * std::cos(pitch) * std::sin(yaw),
                        m_camDist * std::sin(pitch),
                        m_camDist * std::cos(pitch) * std::cos(yaw));
    QMatrix4x4 view;
    view.lookAt(eye, QVector3D(0, 0, 0), QVector3D(0, 1, 0));
    return view;
}

void AttitudeGLWidget::drawBuffer(QOpenGLBuffer &vbo, int vertexCount, unsigned int mode)
{
    if (vertexCount <= 0)
        return;
    vbo.bind();
    m_program.enableAttributeArray(0);
    m_program.enableAttributeArray(1);
    m_program.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(AttitudeVertex));
    m_program.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 3, sizeof(AttitudeVertex));
    glDrawArrays(mode, 0, vertexCount);
    m_program.disableAttributeArray(0);
    m_program.disableAttributeArray(1);
    vbo.release();
}

void AttitudeGLWidget::paintGL()
{
    const QColor bg = palette().color(QPalette::Base);

    if (!m_shaderOk) {
        // GL init failed (shader link error, e.g. a driver that only offers
        // a context too old for even this modest shader): fall back to a
        // plain 2D message instead of a silent black viewport (blocker 6).
        // No native GL drawing at all here -- glClearColor/glClear alone are
        // safe even without a working program, but there is nothing useful
        // left to draw with one.
        QPainter painter(this);
        painter.fillRect(rect(), bg);
        painter.setRenderHint(QPainter::Antialiasing);
        QFont f = painter.font();
        f.setBold(true);
        painter.setFont(f);
        painter.setPen(QColor(200, 60, 60));
        painter.drawText(rect(), Qt::AlignCenter | Qt::TextWordWrap,
                         tr("OpenGL shader failed to initialise -\n3D attitude view unavailable "
                            "(see the debug log for the driver's error)"));
        return;
    }

    QPainter painter(this);
    painter.beginNativePainting();

    glClearColor(float(bg.redF()), float(bg.greenF()), float(bg.blueF()), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // Advance the displayed pose towards the latest sample. Only while live:
    // otherwise the pose stays exactly where it was (frozen), which is what
    // "never present a stale pose as live" means for the model itself -- the
    // shader's uLive uniform desaturates it as the visible tell.
    if (m_haveSample && m_live) {
        const double alpha = qBound(0.0,
            double(m_sinceSample.elapsed()) / double(qMax<qint64>(1, m_sampleIntervalMs)), 1.0);
        m_displayQuat = QQuaternion::slerp(m_prevQuat, m_latestQuat, float(alpha));
    }

    const QMatrix4x4 view = camera();
    const QMatrix4x4 vp = m_proj * view;
    const QMatrix4x4 roomMvp  = vp * nedToGlBasis();
    const QMatrix4x4 modelMvp = vp * attitudeModelMatrix(m_displayQuat);

    m_program.bind();

    // Room geometry (grid + triad) is fixed to NED, never desaturated by the
    // live/stale state -- it is the frame of reference, not the vehicle.
    m_program.setUniformValue("uMvp", roomMvp);
    m_program.setUniformValue("uLive", 1.0f);

    // Grid colour: current palette, recomputed every frame (cheap - two
    // QColor lookups) rather than baked into the VBO, so a live OS theme
    // switch is reflected on the very next repaint instead of only after the
    // widget is recreated (SWE1-GUI-002 review fix).
    const QColor baseC = palette().color(QPalette::Base);
    const QColor fgC   = palette().color(QPalette::WindowText);
    const QVector3D gridColorNow(
        float(baseC.redF()   * 0.75 + fgC.redF()   * 0.25),
        float(baseC.greenF() * 0.75 + fgC.greenF() * 0.25),
        float(baseC.blueF()  * 0.75 + fgC.blueF()  * 0.25));
    m_program.setUniformValue("uUseOverrideColor", 1.0f);
    m_program.setUniformValue("uOverrideColor", gridColorNow);
    drawBuffer(m_gridVbo, m_gridVertCount, GL_LINES);

    // Triad keeps its own fixed, saturated per-vertex colours (axis-gizmo
    // convention, deliberately theme-independent -- see SWE1-GUI-002).
    m_program.setUniformValue("uUseOverrideColor", 0.0f);
    drawBuffer(m_triadVbo, m_triadVertCount, GL_LINES);

    m_program.setUniformValue("uMvp", modelMvp);
    m_program.setUniformValue("uLive", m_live ? 1.0f : 0.0f);
    drawBuffer(m_modelVbo, m_modelVertCount, GL_TRIANGLES);

    m_program.release();
    painter.endNativePainting();

    painter.setRenderHint(QPainter::Antialiasing);
    drawAxisLabels(painter, roomMvp);
}

QPointF AttitudeGLWidget::projectToWidget(const QMatrix4x4 &mvp, const QVector3D &nedPoint) const
{
    const QVector4D clip = mvp * QVector4D(nedPoint, 1.0f);
    if (clip.w() <= 1e-4f)
        return QPointF(-1000, -1000);   // behind the camera: park off-screen
    const QVector3D ndc = clip.toVector3DAffine();
    return QPointF((ndc.x() * 0.5 + 0.5) * width(),
                   (1.0 - (ndc.y() * 0.5 + 0.5)) * height());
}

void AttitudeGLWidget::drawAxisLabels(QPainter &painter, const QMatrix4x4 &roomMvp)
{
    struct Label { QVector3D ned; const char *text; QColor color; };
    const Label labels[] = {
        { QVector3D(1.35f, 0.0f, 0.0f), "N", QColor(230, 100, 100) },
        { QVector3D(0.0f, 1.35f, 0.0f), "E", QColor(110, 210, 130) },
        { QVector3D(0.0f, 0.0f, 1.35f), "D", QColor(120, 170, 235) },
    };
    QFont f = painter.font();
    f.setBold(true);
    painter.setFont(f);
    for (const Label &l : labels) {
        const QPointF pt = projectToWidget(roomMvp, l.ned);
        painter.setPen(l.color);
        painter.drawText(pt, QString::fromLatin1(l.text));
    }
}

void AttitudeGLWidget::mousePressEvent(QMouseEvent *e)
{
    m_lastMousePos = e->pos();
    e->accept();
}

void AttitudeGLWidget::mouseMoveEvent(QMouseEvent *e)
{
    if (e->buttons() & Qt::LeftButton) {
        const QPoint delta = e->pos() - m_lastMousePos;
        m_camYawDeg += float(delta.x()) * 0.4f;
        m_camPitchDeg = qBound(-85.0f, m_camPitchDeg - float(delta.y()) * 0.4f, 85.0f);
        m_lastMousePos = e->pos();
        update();
    }
}

void AttitudeGLWidget::wheelEvent(QWheelEvent *e)
{
    const float steps = float(e->angleDelta().y()) / 120.0f;
    m_camDist = qBound(1.2f, m_camDist - steps * 0.25f, 12.0f);
    update();
    e->accept();
}

void AttitudeGLWidget::buildGeometry()
{
    QVector<AttitudeVertex> model;

    const QVector3D bodyColor(0.30f, 0.32f, 0.36f);
    const QVector3D armColor(0.42f, 0.44f, 0.48f);
    const QVector3D rotorColor(0.08f, 0.08f, 0.09f);
    const QVector3D noseColor(0.95f, 0.32f, 0.06f);

    // Body: x forward/back, y right/left, z down/up (body frame).
    addOrientedBox(model, QVector3D(0, 0, 0),
                  QVector3D(1, 0, 0), 0.14f,
                  QVector3D(0, 1, 0), 0.10f,
                  QVector3D(0, 0, 1), 0.045f,
                  bodyColor);

    // Four arms in an X layout (+/-45 deg off forward), a rotor disc at each tip.
    const float armLen = 0.34f;
    const float rotorRadius = 0.085f;
    const float arms[4][2] = { { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } };
    for (const auto &a : arms) {
        QVector3D dir(a[0], a[1], 0.0f);
        dir.normalize();
        const QVector3D tip = dir * armLen;
        const QVector3D mid = tip * 0.5f;
        const QVector3D widthAxis(dir.y(), -dir.x(), 0.0f);   // horizontal, perpendicular to dir
        addOrientedBox(model, mid,
                      dir, armLen * 0.5f,
                      widthAxis, 0.014f,
                      QVector3D(0, 0, 1), 0.012f,
                      armColor);
        addDisc(model, tip, rotorRadius, 16, rotorColor);
    }

    // Nose marker: small bright pyramid ahead of the body's front face.
    addPyramid(model, QVector3D(0.14f + 0.11f, 0, 0), QVector3D(0.14f, 0, 0), 0.045f, 8, noseColor);

    m_modelVertCount = model.size();
    m_modelVbo.create();
    m_modelVbo.bind();
    m_modelVbo.allocate(model.constData(), model.size() * int(sizeof(AttitudeVertex)));
    m_modelVbo.release();

    // Ground grid: NED plane at D = +1.0 (below the model), N/E in [-2, 2].
    // The colour baked into these vertices is a placeholder and is never
    // actually seen: paintGL() overrides it every frame with a uniform
    // derived fresh from the current QPalette (uUseOverrideColor/
    // uOverrideColor), so a live OS theme switch is reflected immediately
    // instead of only after this VBO is rebuilt (SWE1-GUI-002 review fix --
    // baking the palette blend in here once, at GL-init time, went stale
    // until the widget was recreated).
    QVector<AttitudeVertex> grid;
    const QVector3D gridColor(0.5f, 0.5f, 0.5f);
    constexpr float extent = 2.0f, step = 0.5f, d = 1.0f;
    for (float n = -extent; n <= extent + 1e-3f; n += step) {
        grid.append(vertexOf(QVector3D(n, -extent, d), gridColor));
        grid.append(vertexOf(QVector3D(n, extent, d), gridColor));
    }
    for (float e = -extent; e <= extent + 1e-3f; e += step) {
        grid.append(vertexOf(QVector3D(-extent, e, d), gridColor));
        grid.append(vertexOf(QVector3D(extent, e, d), gridColor));
    }
    m_gridVertCount = grid.size();
    m_gridVbo.create();
    m_gridVbo.bind();
    m_gridVbo.allocate(grid.constData(), grid.size() * int(sizeof(AttitudeVertex)));
    m_gridVbo.release();

    // N/E/D triad, room-fixed (drawn with nedToGlBasis only -- never rotated
    // by the quaternion, since it represents the room, not the vehicle).
    QVector<AttitudeVertex> triad;
    const QVector3D nColor(0.85f, 0.30f, 0.30f), eColor(0.30f, 0.80f, 0.40f), dColor(0.35f, 0.55f, 0.95f);
    triad.append(vertexOf(QVector3D(0, 0, 0), nColor));
    triad.append(vertexOf(QVector3D(1.2f, 0, 0), nColor));
    triad.append(vertexOf(QVector3D(0, 0, 0), eColor));
    triad.append(vertexOf(QVector3D(0, 1.2f, 0), eColor));
    triad.append(vertexOf(QVector3D(0, 0, 0), dColor));
    triad.append(vertexOf(QVector3D(0, 0, 1.2f), dColor));
    m_triadVertCount = triad.size();
    m_triadVbo.create();
    m_triadVbo.bind();
    m_triadVbo.allocate(triad.constData(), triad.size() * int(sizeof(AttitudeVertex)));
    m_triadVbo.release();
}
