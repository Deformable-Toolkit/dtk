/********************************************************************************
** Form generated from reading UI file 'ImageInterface.ui'
**
** Created by: Qt User Interface Compiler version 5.15.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_IMAGEINTERFACE_H
#define UI_IMAGEINTERFACE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_ImageInterface
{
public:
    QVBoxLayout *vboxLayout;
    QGridLayout *gridLayout;
    QSpinBox *imgHeight;
    QSpinBox *imgWidth;
    QLabel *label_2;
    QSpacerItem *spacerItem;
    QLabel *label_3;
    QCheckBox *ratioCheckBox;
    QHBoxLayout *hboxLayout;
    QLabel *label;
    QDoubleSpinBox *oversampling;
    QSpacerItem *spacerItem1;
    QComboBox *color_comboBox;
    QCheckBox *expandFrustum;
    QSpacerItem *spacerItem2;
    QHBoxLayout *hboxLayout1;
    QSpacerItem *spacerItem3;
    QPushButton *okButton;
    QPushButton *cancelButton;

    void setupUi(QDialog *ImageInterface)
    {
        if (ImageInterface->objectName().isEmpty())
            ImageInterface->setObjectName(QString::fromUtf8("ImageInterface"));
        ImageInterface->resize(484, 447);
        vboxLayout = new QVBoxLayout(ImageInterface);
        vboxLayout->setSpacing(6);
        vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
        vboxLayout->setContentsMargins(9, 9, 9, 9);
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        imgHeight = new QSpinBox(ImageInterface);
        imgHeight->setObjectName(QString::fromUtf8("imgHeight"));
        imgHeight->setMinimum(1);
        imgHeight->setMaximum(32000);

        gridLayout->addWidget(imgHeight, 0, 4, 1, 1);

        imgWidth = new QSpinBox(ImageInterface);
        imgWidth->setObjectName(QString::fromUtf8("imgWidth"));
        imgWidth->setMinimum(1);
        imgWidth->setMaximum(32000);

        gridLayout->addWidget(imgWidth, 0, 1, 1, 1);

        label_2 = new QLabel(ImageInterface);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 0, 0, 1, 1);

        spacerItem = new QSpacerItem(20, 22, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(spacerItem, 0, 2, 1, 1);

        label_3 = new QLabel(ImageInterface);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 0, 3, 1, 1);

        ratioCheckBox = new QCheckBox(ImageInterface);
        ratioCheckBox->setObjectName(QString::fromUtf8("ratioCheckBox"));
        ratioCheckBox->setChecked(true);

        gridLayout->addWidget(ratioCheckBox, 1, 4, 1, 1);


        vboxLayout->addLayout(gridLayout);

        hboxLayout = new QHBoxLayout();
        hboxLayout->setSpacing(6);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(ImageInterface);
        label->setObjectName(QString::fromUtf8("label"));

        hboxLayout->addWidget(label);

        oversampling = new QDoubleSpinBox(ImageInterface);
        oversampling->setObjectName(QString::fromUtf8("oversampling"));
        oversampling->setDecimals(1);
        oversampling->setMinimum(0.100000000000000);
        oversampling->setMaximum(64.000000000000000);
        oversampling->setSingleStep(1.000000000000000);
        oversampling->setValue(1.000000000000000);

        hboxLayout->addWidget(oversampling);

        spacerItem1 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout->addItem(spacerItem1);


        vboxLayout->addLayout(hboxLayout);

        color_comboBox = new QComboBox(ImageInterface);
        color_comboBox->addItem(QString());
        color_comboBox->addItem(QString());
        color_comboBox->addItem(QString());
        color_comboBox->setObjectName(QString::fromUtf8("color_comboBox"));

        vboxLayout->addWidget(color_comboBox);

        expandFrustum = new QCheckBox(ImageInterface);
        expandFrustum->setObjectName(QString::fromUtf8("expandFrustum"));

        vboxLayout->addWidget(expandFrustum);

        spacerItem2 = new QSpacerItem(20, 16, QSizePolicy::Minimum, QSizePolicy::Expanding);

        vboxLayout->addItem(spacerItem2);

        hboxLayout1 = new QHBoxLayout();
        hboxLayout1->setSpacing(6);
        hboxLayout1->setObjectName(QString::fromUtf8("hboxLayout1"));
        hboxLayout1->setContentsMargins(0, 0, 0, 0);
        spacerItem3 = new QSpacerItem(131, 31, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout1->addItem(spacerItem3);

        okButton = new QPushButton(ImageInterface);
        okButton->setObjectName(QString::fromUtf8("okButton"));

        hboxLayout1->addWidget(okButton);

        cancelButton = new QPushButton(ImageInterface);
        cancelButton->setObjectName(QString::fromUtf8("cancelButton"));

        hboxLayout1->addWidget(cancelButton);


        vboxLayout->addLayout(hboxLayout1);


        retranslateUi(ImageInterface);
        QObject::connect(okButton, SIGNAL(clicked()), ImageInterface, SLOT(accept()));
        QObject::connect(cancelButton, SIGNAL(clicked()), ImageInterface, SLOT(reject()));

        QMetaObject::connectSlotsByName(ImageInterface);
    } // setupUi

    void retranslateUi(QDialog *ImageInterface)
    {
        ImageInterface->setWindowTitle(QCoreApplication::translate("ImageInterface", "Image Settings", nullptr));
#if QT_CONFIG(tooltip)
        imgHeight->setToolTip(QCoreApplication::translate("ImageInterface", "Height of the image (in pixels)", nullptr));
#endif // QT_CONFIG(tooltip)
        imgHeight->setSuffix(QCoreApplication::translate("ImageInterface", " px", nullptr));
#if QT_CONFIG(tooltip)
        imgWidth->setToolTip(QCoreApplication::translate("ImageInterface", "Width of the image (in pixels)", nullptr));
#endif // QT_CONFIG(tooltip)
        imgWidth->setSuffix(QCoreApplication::translate("ImageInterface", " px", nullptr));
        label_2->setText(QCoreApplication::translate("ImageInterface", "Width", nullptr));
        label_3->setText(QCoreApplication::translate("ImageInterface", "Height", nullptr));
        ratioCheckBox->setText(QCoreApplication::translate("ImageInterface", "Keep Ratio", nullptr));
        label->setText(QCoreApplication::translate("ImageInterface", "Oversampling", nullptr));
#if QT_CONFIG(tooltip)
        oversampling->setToolTip(QCoreApplication::translate("ImageInterface", "Antialiases image (when larger then 1.0)", nullptr));
#endif // QT_CONFIG(tooltip)
        oversampling->setPrefix(QCoreApplication::translate("ImageInterface", "x ", nullptr));
        color_comboBox->setItemText(0, QCoreApplication::translate("ImageInterface", "Use current background color", nullptr));
        color_comboBox->setItemText(1, QCoreApplication::translate("ImageInterface", "Use transparent background", nullptr));
        color_comboBox->setItemText(2, QCoreApplication::translate("ImageInterface", "Choose background color", nullptr));

#if QT_CONFIG(tooltip)
        expandFrustum->setToolTip(QCoreApplication::translate("ImageInterface", "When image aspect ratio differs from viewer's one, expand frustum as needed. Fits inside current frustum otherwise.", nullptr));
#endif // QT_CONFIG(tooltip)
        expandFrustum->setText(QCoreApplication::translate("ImageInterface", "Expand Frustum if Needed", nullptr));
        okButton->setText(QCoreApplication::translate("ImageInterface", "OK", nullptr));
        cancelButton->setText(QCoreApplication::translate("ImageInterface", "Cancel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ImageInterface: public Ui_ImageInterface {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_IMAGEINTERFACE_H
