#include "themedata.h"

// 暗主题：中性炭灰
const ThemeData themeDark{
    "#0a0a0a",    // bgPrimary
    "#121212",    // bgSurface
    "#181818",    // bgControl
    "#202020",    // bgElevated
    "#2a2a2a",    // border
    "#3a3a3a",    // borderLight
    "#d4d4d4",    // textPrimary
    "#888888",    // textSecondary
    "#666666",    // textMuted
    "#4ec9b0",    // accentGreen
    "#0d1f1b",    // accentGreenBg
    "#cc9a2a",    // accentOrange
    "#f14c4c",    // accentRed
    "#4ec9b0",    // btnOn
    "#0d1f1b",    // btnOnBg
    "#4ec9b0",    // btnOnBorder
    "#666666",    // btnOff
    "#181818",    // btnOffBg
    "#2a2a2a",    // btnOffBorder
};

// 亮主题：极简浅灰
const ThemeData themeLight{
    "#f2f2f2",    // bgPrimary
    "#ffffff",    // bgSurface
    "#f5f5f5",    // bgControl
    "#eaeaea",    // bgElevated
    "#d4d4d4",    // border
    "#e0e0e0",    // borderLight
    "#1a1a1a",    // textPrimary
    "#555555",    // textSecondary
    "#888888",    // textMuted
    "#2ea043",    // accentGreen
    "#daf1dd",    // accentGreenBg
    "#b8860b",    // accentOrange
    "#d32f2f",    // accentRed
    "#2ea043",    // btnOn
    "#daf1dd",    // btnOnBg
    "#2ea043",    // btnOnBorder
    "#999999",    // btnOff
    "#f5f5f5",    // btnOffBg
    "#d4d4d4",    // btnOffBorder
};

const ThemeData *currentTheme = &themeDark;

// 全局 QSS 生成
QString ThemeData::toStyleSheet() const
{
    return QString(R"(
        QMainWindow,QWidget{background:%1;color:%2;font-size:16px;}

        QGroupBox{
          background:%3;
          border:1px solid %5;
          border-radius:2px;
          margin-top:20px;
          padding:14px 10px 10px 10px;
        }
        QGroupBox::title{
          subcontrol-origin:margin;
          subcontrol-position:top left;
          left:10px;
          padding:0;
          color:%8;
          font-size:15px;
          font-weight:500;
          background:%3;
          border:none;
        }

        QLabel{color:%8;}
        QLabel#sectionTitle{font-weight:600;font-size:15px;color:%8;padding:4px 0 2px 0;}

        QPushButton{
          background:%4;
          border:1px solid %5;
          border-radius:2px;
          color:%7;
          padding:4px 10px;
          outline:none;
        }
        QPushButton:hover{background:%6;border-color:%9;}
        QPushButton:pressed{background:%3;}
        QPushButton[state="on"]{
          color:%10;background:%11;border:1px solid %12;font-weight:bold;
        }
        QPushButton[state="on"]:hover{
          background:%6;border-color:%10;
        }
        QPushButton[state="off"]{
          color:%13;background:%14;border:1px solid %15;
        }
        QPushButton[state="off"]:hover{
          background:%6;border-color:%9;
        }

        QLineEdit{
          background:%4;
          border:1px solid %5;
          border-radius:2px;
          color:%7;
          padding:4px 8px;
        }

        QComboBox{
          background:%4;
          border:1px solid %5;
          border-radius:2px;
          color:%7;
          padding:4px 8px;
        }
        QComboBox:hover{border-color:%9;}
        QComboBox::drop-down{border:1px solid %5;width:18px;}
        QComboBox::down-arrow{
          border-left:4px solid transparent;
          border-right:4px solid transparent;
          border-top:5px solid %8;
          width:0px;height:0px;
          margin-right:2px;
        }


        QListWidget{
          background:%3;
          border:1px solid %5;
          border-radius:2px;
          color:%7;
          outline:none;
        }
        QListWidget::item{padding:3px 6px;}
        QListWidget::item:hover{background:%4;}
        QListWidget::item:selected{background:%6;color:%7;}

        QTextEdit{
          background:%1;
          border:1px solid %5;
          border-radius:2px;
          color:%7;
          padding:4px;
        }
        QTextEdit#hexDisplay{color:%16;font-family:Consolas;font-size:14px;}
        QTextEdit#bitDisplay{color:%8;font-family:Consolas;font-size:14px;}
        QTextEdit#logDisplay{color:%8;font-family:Consolas;font-size:14px;}
        QTextEdit#hexDisplay:focus,QTextEdit#bitDisplay:focus,QTextEdit#logDisplay:focus{
          outline:none;
        }
        QLabel#refChannel{font-weight:bold;color:%17;}
        QLabel#refChLabel{background:transparent;}
        QLabel#hexLabel,QLabel#binLabel,QLabel#refLabel,QLabel#ifLabel,QLabel#logLabel,QLabel#sectionTitle{
          background:transparent;border:none;
        }
        QComboBox#refChannelCombo{
          background:transparent;border:none;color:%7;padding:2px 4px;
        }
        QComboBox#refChannelCombo:hover{color:%16;}
        QLabel#antName{
          border:2px solid %17;
          border-radius:11px;
          font-weight:bold;font-size:15px;
          color:%7;
          background:transparent;
        }
        QWidget#antBlock{
          border:1px solid %5;
          border-radius:4px;
          background:%4;
        }
        QGroupBox#rightPanel{padding:8px 10px 10px 10px;}

        QWidget#vAntConnector{
          border-left:1px dashed %17;
          background:transparent;
          min-width:1px;max-width:1px;
        }
        QWidget#hAntConnector{
          border-top:1px dashed %17;
          background:transparent;
          min-height:1px;max-height:1px;
        }
        QWidget#cAntConnector{
          border-left:1px dashed %17;
          border-top:1px dashed %17;
          background:transparent;
          min-width:1px;min-height:1px;max-width:1px;max-height:1px;
        }
        QPushButton#sendBtn{font-weight:bold;color:%16;}
        QPushButton#themeBtn{font-size:14px;padding:2px 8px;}
        QLabel#statusLabel{font-weight:bold;color:%18;}
        QLabel#statusLabel[connected="true"]{font-weight:bold;color:%16;}

        QScrollBar:vertical{background:transparent;width:6px;border:none;}
        QScrollBar::handle:vertical{background:%5;min-height:24px;border-radius:3px;}
        QScrollBar::handle:vertical:hover{background:%9;}
        QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0;}
        QScrollBar::add-page:vertical,QScrollBar::sub-page:vertical{background:none;}

        QSlider::groove:horizontal{background:%6;height:2px;border-radius:1px;}
        QSlider::handle:horizontal{background:%9;width:10px;height:10px;margin:-4px 0;border-radius:5px;}
        QSlider::handle:horizontal:hover{background:%8;}
        QSlider::sub-page:horizontal{background:%10;border-radius:1px;}

        QSpinBox{
          background:%4;
          border:1px solid %5;
          border-radius:2px;
          color:%7;
          padding:2px 4px;
        }
        QSpinBox::up-button,QSpinBox::down-button{
          background:%4;
          border:1px solid %5;
          width:14px;
        }
        QSpinBox::up-button:hover,QSpinBox::down-button:hover{
          background:%6;
        }
        QSpinBox::up-arrow{
          border-left:4px solid transparent;
          border-right:4px solid transparent;
          border-bottom:5px solid %8;
          width:0px;height:0px;
        }
        QSpinBox::down-arrow{
          border-left:4px solid transparent;
          border-right:4px solid transparent;
          border-top:5px solid %8;
          width:0px;height:0px;
        }


        QCheckBox{color:%7;spacing:4px;}
        QCheckBox::indicator{
          width:14px;height:14px;
          border:1px solid %9;
          border-radius:2px;
          background:%4;
        }
        QCheckBox::indicator:checked{background:%9;border-color:%9;}

        QToolBar{
          background:%3;
          border-bottom:1px solid %5;
          padding:4px 8px;spacing:6px;
        }

        QSplitter::handle{background:%1;}
        QSplitter::handle:hover{background:%5;}

        QToolTip{
          background:%4;
          border:1px solid %5;
          border-radius:2px;
          color:%7;
          padding:4px 8px;
          font-size:15px;
        }
    )")
    .arg(bgPrimary).arg(textPrimary)
    .arg(bgSurface).arg(bgControl)
    .arg(border).arg(bgElevated)
    .arg(textPrimary).arg(textSecondary)
    .arg(borderLight)
    .arg(btnOn).arg(btnOnBg).arg(btnOnBorder)
    .arg(btnOff).arg(btnOffBg).arg(btnOffBorder)
    .arg(accentGreen).arg(accentOrange)
    .arg(accentRed);
}
