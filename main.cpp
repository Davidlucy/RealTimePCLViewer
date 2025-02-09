#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "pcl_reader/pclreader.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    qmlRegisterType<PCLReader>("PCDReader", 1, 0, "PCLReader");

    QQmlApplicationEngine engine;
    const QUrl url(u"qrc:/testUDP/main.qml"_qs);
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
        &app, [url](QObject *obj, const QUrl &objUrl) {
            if (!obj && url == objUrl)
                QCoreApplication::exit(-1);
        }, Qt::QueuedConnection);
    engine.load(url);

    return app.exec();
}
