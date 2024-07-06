#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Cylinder.h>
#include <Magnum/Shaders/PhongGL.h>
#include <Magnum/Trade/MeshData.h>
#include <vector>

using namespace Magnum;
using namespace Math::Literals;


class PrimitivesExample : public Platform::Application {
public:
    explicit PrimitivesExample(const Arguments &arguments);

private:
    void drawEvent() override;

    void mousePressEvent(MouseEvent &event) override;

    void mouseReleaseEvent(MouseEvent &event) override;

    void mouseMoveEvent(MouseMoveEvent &event) override;

    GL::Mesh _mesh;
    Shaders::PhongGL _shader;

    Matrix4 _transformation, _projection;
    Color3 _color;
};


PrimitivesExample::PrimitivesExample(const Arguments &arguments) :
        Platform::Application{arguments, Configuration{}
                .setTitle("Magnum Primitives Example")} {
//    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
//    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
//    _mesh = MeshTools::compile(Primitives::cubeSolid());
    auto data = Primitives::cylinderSolid(100, 30, 5.0);

    data = MeshTools::transform3D(data, Matrix4::scaling({0.3f, 0.3f, 0.3f}));

    _mesh = MeshTools::compile(data);

    _transformation =
            Matrix4::rotationX(30.0_degf) * Matrix4::rotationY(40.0_degf);
    _projection =
            Matrix4::perspectiveProjection(
                    35.0_degf, Vector2{windowSize()}.aspectRatio(), 0.01f, 100.0f) *
            Matrix4::translation(Vector3::zAxis(-10.0f));
    _color = Color3::fromHsv({35.0_degf, 1.0f, 1.0f});
}

void PrimitivesExample::drawEvent() {
    GL::defaultFramebuffer.clear(
        GL::FramebufferClear::Color|GL::FramebufferClear::Depth);

    _shader.setLightPositions({{1.4f, 1.0f, 0.75f, 0.0f}})
        .setDiffuseColor(_color)
        .setAmbientColor(Color3::fromHsv({_color.hue(), 1.0f, 0.3f}))
        .setTransformationMatrix(_transformation)
        .setNormalMatrix(_transformation.normalMatrix())
        .setProjectionMatrix(_projection)
        .draw(_mesh);

    swapBuffers();
}


void PrimitivesExample::mousePressEvent(MouseEvent& event) {
    if(event.button() != MouseEvent::Button::Left) return;

    event.setAccepted();
}

void PrimitivesExample::mouseReleaseEvent(MouseEvent& event) {
    _color = Color3::fromHsv({_color.hue() + 50.0_degf, 1.0f, 1.0f});

    event.setAccepted();
    redraw();
}

void PrimitivesExample::mouseMoveEvent(MouseMoveEvent& event) {
    if(!(event.buttons() & MouseMoveEvent::Button::Left)) return;

    Vector2 delta = 3.0f*Vector2{event.relativePosition()}/Vector2{windowSize()};

    _transformation =
        Matrix4::rotationX(Rad{delta.y()})*
        _transformation*
        Matrix4::rotationY(Rad{delta.x()});

    event.setAccepted();
    redraw();
}



MAGNUM_APPLICATION_MAIN(PrimitivesExample)
