// magnumRenderer.h
#ifndef MAGNUMRENDERER_H
#define MAGNUMRENDERER_H

#include <Magnum/Image.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Grid.h>
#include <Magnum/Primitives/Cylinder.h>
#include <Magnum/Primitives/Capsule.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/FlatGL.h>
#include <Magnum/Shaders/VertexColorGL.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/Trade/MeshData.h>
#include "derSimulationEnvironment.h"

namespace Magnum {

using namespace Math::Literals;
using Object3D = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;
using Scene3D = SceneGraph::Scene<SceneGraph::MatrixTransformation3D>;

class magnumRenderer : public Platform::Application, public derSimulationEnvironment {
public:
    explicit magnumRenderer(const shared_ptr<world>& m_world, const simParams& sim_params,
                   const shared_ptr<worldLogger>& logger, int argc, char **argv);
//    explicit magnumRenderer(const Arguments &arguments);
    GL::Mesh _cylinder{NoCreate};

    void runSimulation() override;

private:
    Float depthAt(const Vector2i &windowPosition);
    Vector3 unproject(const Vector2i &windowPosition, Float depth) const;

    void keyPressEvent(KeyEvent &event) override;
    void mousePressEvent(MouseEvent &event) override;
    void mouseMoveEvent(MouseMoveEvent &event) override;
    void mouseScrollEvent(MouseScrollEvent &event) override;
    void drawEvent() override;
//    void tickEvent() override;

    Shaders::VertexColorGL3D _vertexColorShader{NoCreate};
    Shaders::FlatGL3D _flatShader{NoCreate};
    GL::Mesh _mesh{NoCreate}, _grid{NoCreate};
    Shaders::PhongGL _shader{NoCreate};

    Scene3D _scene;
    SceneGraph::DrawableGroup3D _drawables;
    Object3D *_cameraObject;
    SceneGraph::Camera3D *_camera;
    Object3D *cylinder;

    Float _lastDepth;
    Vector2i _lastPosition{-1};
    Vector3 _rotationPoint, _translationPoint;
};

class VertexColorDrawable : public SceneGraph::Drawable3D {
public:
    explicit VertexColorDrawable(Object3D &object, Shaders::VertexColorGL3D &shader, GL::Mesh &mesh,
                                 SceneGraph::DrawableGroup3D &drawables);

    void draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) override;

private:
    Shaders::VertexColorGL3D &_shader;
    GL::Mesh &_mesh;
};

class FlatDrawable : public SceneGraph::Drawable3D {
public:
    explicit FlatDrawable(Object3D &object, Shaders::FlatGL3D &shader, GL::Mesh &mesh,
                          SceneGraph::DrawableGroup3D &drawables);

    void draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) override;

private:
    Shaders::FlatGL3D &_shader;
    GL::Mesh &_mesh;
};

class CylinderDrawable : public SceneGraph::Drawable3D {
public:
    explicit CylinderDrawable(Object3D &object, Shaders::PhongGL &shader, GL::Mesh &mesh,
                              SceneGraph::DrawableGroup3D &drawables);

    void draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) override;

private:
    Shaders::PhongGL &_shader;
    GL::Mesh &_mesh;
    Color3 _color;
};

} // namespace Magnum

#endif // MAGNUMRENDERER_H
