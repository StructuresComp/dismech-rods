// magnumDERSimulationEnvironment.h
#ifndef MAGNUMRENDERER_H
#define MAGNUMRENDERER_H

#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
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
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Primitives/Capsule.h>
#include <Magnum/Primitives/Axis.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/FlatGL.h>
#include <Magnum/Shaders/VertexColorGL.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/PixelFormat.h>
#include <MagnumPlugins/PngImageConverter/PngImageConverter.h>
#include <Corrade/PluginManager/Manager.h>
#include <sstream>
#include <iomanip>
#include "derSimulationEnvironment.h"

namespace Magnum {

using namespace Math::Literals;
using Object3D = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;
using Scene3D = SceneGraph::Scene<SceneGraph::MatrixTransformation3D>;


class CapsuleDrawable : public SceneGraph::Drawable3D {
public:
    explicit CapsuleDrawable(Object3D &object, Shaders::PhongGL &shader, GL::Mesh &mesh,
                             SceneGraph::DrawableGroup3D &drawables, Color3 color);

    void draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) override;

private:
    Shaders::PhongGL &shader;
    GL::Mesh &mesh;
    Color3 color;
};


class FlatDrawable : public SceneGraph::Drawable3D {
public:
    explicit FlatDrawable(Object3D &object, Shaders::FlatGL3D &shader, GL::Mesh &mesh,
                          SceneGraph::DrawableGroup3D &drawables, Color3 color);

    void draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) override;

private:
    Shaders::FlatGL3D &shader;
    GL::Mesh &mesh;
    Color3 color;
};


class CoordinateFrame : public SceneGraph::Drawable3D {
public:
    explicit CoordinateFrame(Object3D &object, Shaders::VertexColorGL3D &shader,
                             SceneGraph::DrawableGroup3D &drawables);

    void draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) override;

private:
    Shaders::VertexColorGL3D &shader;
    GL::Mesh mesh;
};


class magnumDERSimulationEnvironment : public Platform::Application, public derSimulationEnvironment {
public:
    explicit magnumDERSimulationEnvironment(const shared_ptr<world>& m_world, const renderParams& render_params,
                   const shared_ptr<worldLogger>& logger, int argc, char **argv);

    void runSimulation() override;

private:
    Float depthAt(const Vector2i &windowPosition);
    Vector3 unproject(const Vector2i &windowPosition, Float depth) const;

    float render_scale;
    int render_per;
    string render_record_path;

    void keyPressEvent(KeyEvent &event) override;
    void mousePressEvent(MouseEvent &event) override;
    void mouseMoveEvent(MouseMoveEvent &event) override;
    void mouseScrollEvent(MouseScrollEvent &event) override;
    void drawEvent() override;

    void renderEdge(Eigen::Vector3d top_vertex, Eigen::Vector3d bot_vertex, float radius, Color3 color);

    Shaders::VertexColorGL3D vertex_color_shader{NoCreate};
    Shaders::FlatGL3D flat_shader{NoCreate};
    Shaders::PhongGL phong_shader{NoCreate};

    GL::Mesh capsule_mesh{NoCreate};
    GL::Mesh grid_mesh{NoCreate};
    GL::Mesh floor_mesh{NoCreate};

    Scene3D scene;
    SceneGraph::DrawableGroup3D drawables;
    std::unique_ptr<Object3D> camera_object;
    std::unique_ptr<SceneGraph::Camera3D> camera;

    std::unique_ptr<Object3D> floor;
    std::unique_ptr<Object3D> grid;
    std::unique_ptr<Object3D> coordinate_frame;

    std::vector<std::unique_ptr<Object3D>> edges;
    std::vector<std::unique_ptr<CapsuleDrawable>> capsule_drawables;
    std::vector<std::unique_ptr<SceneGraph::Drawable3D>> misc_drawables;

    Float last_depth;
    Vector2i last_position{-1};
    Vector3 rotation_point, translation_point;
};

} // namespace Magnum

#endif // MAGNUMRENDERER_H
