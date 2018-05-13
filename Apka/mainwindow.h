#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#pragma once
#include <QMainWindow>
#include <QDebug>
#include <QString>
#include <QFile>
#include <QtEndian>
#include <QFileDialog>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_Kompresuj_clicked();

    void on_Dekompresuj_clicked();

private:
void MainWindow::WavReader (const char* fileName, const char* fileToSave);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
