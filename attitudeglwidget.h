#ifndef ATTITUDEGLWIDGET_H
#define ATTITUDEGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QElapsedTimer>
#include <QTimer>

// One coloured triangle/line vertex: position + colour, interleaved, the
// layout the shader's two attributes read straight out of.
struct AttitudeVertex { float x, y, z, r, g, b; };

// Body-to-NED (N forward-of-camera "into the room", E right, D down) mapped
// onto the render scene's basis. A single, reusable function rather than
// inlined per call site -- see the class comment on AttitudeGLWidget for the
// derivation and attitudeSelfCheck() below for the handedness proof.
QMatrix4x4 nedToGlBasis();

// Model matrix for a body-frame vertex: rotate by the body-to-NED quaternion,
// then apply the fixed NED->scene basis.
QMatrix4x4 attitudeModelMatrix(const QQuaternion &bodyToNed);

// Exercises three canonical quaternions (nose-up, right-side-down, yaw+90)
// against their expected screen-space direction (SWE1-GUI-001). Pure math,
// no GL context needed -- safe to run at any time, including at startup.
// *report, if given, gets a human-readable pass/fail table.
bool attitudeSelfCheck(QString *report = nullptr);

// The "Attitude" tab's 3D viewport: a low-poly quadrocopter model rotated by
// the firmware's body-to-NED attitude quaternion, a ground grid and a fixed
// N/E/D axis triad (SYS2-GUI-001), updated live with SLERP between samples
// at display rate (SYS2-GUI-002).
//
// FRAME. Body is NED: x forward, y right, z down (AurixTricore/src/bsw/Ahrs.h
// lines 20-34). The scene's basis is chosen so that N points into the room,
// E to the right and D down on screen -- i.e. GL.x = E, GL.y = -D, GL.z = -N.
// That mapping is a proper rotation (determinant +1, no mirroring): NED is
// right-handed (N x E = D) and so is the GL world basis built this way
// (E x -D = -N), so handedness is preserved end to end from the firmware's
// quaternion to the screen. See attitudeSelfCheck() for the numeric proof.
//
// Nothing here ever does I/O or blocks: pushQuaternion()/setLive() are fed
// already-decoded samples from AttitudeView, and paintGL() only interpolates
// between quaternions already handed to it -- the ~60 Hz repaint timer is the
// only thing driving a frame, never network or file activity.
class AttitudeGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit AttitudeGLWidget(QWidget *parent = nullptr);
    ~AttitudeGLWidget() override;

    // Newest body-to-NED quaternion (w,x,y,z) from a DAQ sample. NaN/degenerate
    // input is ignored outright -- the model holds its last good pose rather
    // than rendering garbage.
    void pushQuaternion(float w, float x, float y, float z);

    // Whether the model should keep tracking new samples (AttState == running
    // and the GUI is connected) or hold and desaturate its last pose.
    void setLive(bool live);

    // Result of attitudeSelfCheck(), cached at construction (pure math, needs
    // no GL context). AttitudeView reads this to show a visible failure
    // banner and to refuse to go live -- SWE1-GUI-001 requires the check to
    // be unconditional (every build, not just debug) and visible in the UI,
    // not just qDebug, since this app ships WIN32_EXECUTABLE with no console.
    bool selfCheckPassed() const { return m_selfCheckOk; }

public slots:
    void resetView();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    void showEvent(QShowEvent *e) override;
    void hideEvent(QHideEvent *e) override;

private:
    void buildGeometry();
    void drawBuffer(QOpenGLBuffer &vbo, int vertexCount, unsigned int mode);
    QMatrix4x4 camera() const;
    QPointF projectToWidget(const QMatrix4x4 &mvp, const QVector3D &nedPoint) const;
    void drawAxisLabels(class QPainter &painter, const QMatrix4x4 &roomMvp);
    // Starts/stops m_renderTimer: only ever ticking while live and visible,
    // never a free-running timer the widget can't see the point of.
    void updateTimerState();

    QOpenGLShaderProgram m_program;
    QOpenGLBuffer m_modelVbo{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_gridVbo{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_triadVbo{QOpenGLBuffer::VertexBuffer};
    int m_modelVertCount = 0;
    int m_gridVertCount  = 0;
    int m_triadVertCount = 0;
    QMatrix4x4 m_proj;

    // Interpolation state: the display pose SLERPs from m_prevQuat (wherever
    // it currently is) towards m_latestQuat (the newest sample) over
    // m_sampleIntervalMs, measured from the samples actually arriving rather
    // than hardcoded to the nominal 100 ms -- so it stays correct if a future
    // firmware strand changes the DAQ rate.
    QQuaternion   m_prevQuat{1.0f, 0.0f, 0.0f, 0.0f};
    QQuaternion   m_latestQuat{1.0f, 0.0f, 0.0f, 0.0f};
    QQuaternion   m_displayQuat{1.0f, 0.0f, 0.0f, 0.0f};
    qint64        m_sampleIntervalMs = 100;
    QElapsedTimer m_sinceSample;
    bool          m_haveSample = false;
    bool          m_live       = false;
    bool          m_selfCheckOk = false;   // see selfCheckPassed()
    bool          m_shaderOk    = false;   // false: paintGL shows a text fallback, no GL draw

    QTimer m_renderTimer;   // ~60 Hz repaint; only ticks while live and visible

    // Orbit camera: mouse-drag orbit, wheel zoom, "reset view" back to this.
    static constexpr float kDefaultYawDeg   = -35.0f;
    static constexpr float kDefaultPitchDeg = 22.0f;
    static constexpr float kDefaultDist     = 3.2f;
    float  m_camYawDeg   = kDefaultYawDeg;
    float  m_camPitchDeg = kDefaultPitchDeg;
    float  m_camDist     = kDefaultDist;
    QPoint m_lastMousePos;
};

#endif // ATTITUDEGLWIDGET_H
