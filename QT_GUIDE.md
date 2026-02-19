# Reading Real-Time CSV Data in Qt (C++)

This guide explains how to read the CSV telemetry stream from the STM32 via USB CDC using the Qt Framework (`QSerialPort`).

## 1. Project Setup (.pro)

Add the `serialport` module to your `.pro` file:

```pro
QT += serialport
```

## 2. Header File (`mainwindow.h`)

Include `QSerialPort` and define the necessary slots.

```cpp
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btnConnect_clicked();
    void readSerialData(); // Slot to handle incoming data

private:
    Ui::MainWindow *ui;
    QSerialPort *serial;
    void processLine(const QByteArray &line);
};

#endif // MAINWINDOW_H
```

## 3. Source File (`mainwindow.cpp`)

Implement the connection logic and the `readyRead` handler. The key is to read line-by-line using `canReadLine()`.

```cpp
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serial(new QSerialPort(this))
{
    ui->setupUi(this);

    // Fill combo box with available ports
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        ui->cmbPorts->addItem(info.portName());
    }

    // Connect the readyRead signal to our slot
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::readSerialData);
}

MainWindow::~MainWindow()
{
    if (serial->isOpen())
        serial->close();
    delete ui;
}

void MainWindow::on_btnConnect_clicked()
{
    if (serial->isOpen()) {
        serial->close();
        ui->btnConnect->setText("Connect");
        return;
    }

    serial->setPortName(ui->cmbPorts->currentText());
    serial->setBaudRate(QSerialPort::Baud115200); // Baud rate is virtual for USB CDC, but good practice
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (serial->open(QIODevice::ReadOnly)) {
        ui->btnConnect->setText("Disconnect");
        qDebug() << "Connected to" << serial->portName();
    } else {
        QMessageBox::critical(this, "Error", serial->errorString());
    }
}

void MainWindow::readSerialData()
{
    // Read all available lines
    while (serial->canReadLine()) {
        QByteArray line = serial->readLine().trimmed(); // Remove \r\n
        processLine(line);
    }
}

void MainWindow::processLine(const QByteArray &line)
{
    // Skip empty lines
    if (line.isEmpty()) return;

    // Convert to string
    QString strLine = QString::fromUtf8(line);

    // Check for Header (starts with "t_ms")
    if (strLine.startsWith("t_ms")) {
        qDebug() << "Header received:" << strLine;
        return;
    }

    // Split CSV
    QStringList parts = strLine.split(',');

    // Ensure we have the expected number of columns (15 in this case)
    if (parts.size() < 15) {
        // Incomplete line or different format
        return;
    }

    // Parse specific fields (Example)
    bool ok;
    quint32 t_ms = parts[0].toUInt(&ok);
    qint32 roll_mdeg = parts[4].toInt(&ok);  // Filtered Roll (milli-degrees)
    qint32 pwm_cmd = parts[10].toInt(&ok);   // PWM Command (x100)

    if (!ok) return; // Parsing failed

    double roll_deg = roll_mdeg / 1000.0;
    double pwm = pwm_cmd / 100.0;

    // Update UI (Example: Labels)
    ui->lblTime->setText(QString::number(t_ms));
    ui->lblRoll->setText(QString::number(roll_deg, 'f', 2));
    ui->lblPWM->setText(QString::number(pwm, 'f', 2));

    // Plotting?
    // You can pass 't_ms' and 'roll_deg' to a QCustomPlot widget here.
    qDebug() << "Time:" << t_ms << " Roll:" << roll_deg << " PWM:" << pwm;
}
```

## 4. UI Setup (`mainwindow.ui`)

Ensure you have:
- A `QComboBox` named `cmbPorts`.
- A `QPushButton` named `btnConnect`.
- `QLabel`s named `lblTime`, `lblRoll`, `lblPWM` for display.

## Key Considerations

1.  **Buffering:** `QSerialPort` buffers data automatically. Using `canReadLine()` ensures you process complete lines, which is critical for CSV data integrity.
2.  **Threading:** For high-frequency data (like 1kHz), consider moving the serial reading to a separate thread or using a high-performance plotting library (like `QCustomPlot` with its own optimized data containers). For 50Hz (20ms), the main GUI thread is usually fine.
3.  **Data Format:** The embedded code sends integers scaled by 1000 or 100. You **must** divide by 1000.0 or 100.0 in Qt to get the actual float values.
    *   `roll_filt` (index 4) -> Divide by 1000.0
    *   `p`, `i`, `d`, `output` -> Divide by 1000.0
    *   `pwm_cmd`, `pwm_sat` -> Divide by 100.0
