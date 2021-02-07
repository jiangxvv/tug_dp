/********************************************************************************
** Form generated from reading UI file 'demoI32517.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef DEMOI32517_H
#define DEMOI32517_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_DpMonitor
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *velLineEdit;
    QLabel *label_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_3;
    QLineEdit *dirLineEdit;
    QLabel *label_4;

    void setupUi(QWidget *DpMonitor)
    {
        if (DpMonitor->objectName().isEmpty())
            DpMonitor->setObjectName(QStringLiteral("DpMonitor"));
        DpMonitor->resize(207, 84);
        gridLayout = new QGridLayout(DpMonitor);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(DpMonitor);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        velLineEdit = new QLineEdit(DpMonitor);
        velLineEdit->setObjectName(QStringLiteral("velLineEdit"));
        velLineEdit->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        velLineEdit->setReadOnly(true);

        horizontalLayout->addWidget(velLineEdit);

        label_2 = new QLabel(DpMonitor);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout->addWidget(label_2);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_3 = new QLabel(DpMonitor);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_2->addWidget(label_3);

        dirLineEdit = new QLineEdit(DpMonitor);
        dirLineEdit->setObjectName(QStringLiteral("dirLineEdit"));
        dirLineEdit->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        dirLineEdit->setReadOnly(true);

        horizontalLayout_2->addWidget(dirLineEdit);

        label_4 = new QLabel(DpMonitor);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout_2->addWidget(label_4);


        verticalLayout->addLayout(horizontalLayout_2);


        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);


        retranslateUi(DpMonitor);

        QMetaObject::connectSlotsByName(DpMonitor);
    } // setupUi

    void retranslateUi(QWidget *DpMonitor)
    {
        DpMonitor->setWindowTitle(QApplication::translate("DpMonitor", "DP Montor", 0));
        label->setText(QApplication::translate("DpMonitor", "Speed:    ", 0));
        label_2->setText(QApplication::translate("DpMonitor", "m/s", 0));
        label_3->setText(QApplication::translate("DpMonitor", "Heading:", 0));
        label_4->setText(QApplication::translate("DpMonitor", "deg", 0));
    } // retranslateUi

};

namespace Ui {
    class DpMonitor: public Ui_DpMonitor {};
} // namespace Ui

QT_END_NAMESPACE

#endif // DEMOI32517_H
