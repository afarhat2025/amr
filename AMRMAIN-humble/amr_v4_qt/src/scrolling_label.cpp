#include "scrolling_label.h"
#include <QPainter>
#include <QFontMetrics>

ScrollingLabel::ScrollingLabel(QWidget *parent)
    : QLabel(parent), offset(0)
{
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    setTextInteractionFlags(Qt::NoTextInteraction);
    setWordWrap(false);  // no wrapping — we scroll horizontally
    setAlignment(Qt::AlignVCenter);

    connect(&scrollTimer, &QTimer::timeout, this, [=]() {
        offset += 2;  // speed of scrolling
        update();
    });
    scrollTimer.start(30);  // 30 ms per frame (~33 FPS)
}

void ScrollingLabel::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    QFontMetrics fm(font());
    int textWidth = fm.horizontalAdvance(text());
    int labelWidth = width();

    if (textWidth <= labelWidth) {
        painter.drawText(rect(), alignment(), text());
        return;
    }

    int x = -offset % (textWidth + labelWidth);
    painter.drawText(x, height() / 2 + fm.ascent() / 2, text());

    // Repeat drawing for seamless scroll
    if (x + textWidth < labelWidth)
        painter.drawText(x + textWidth + 20, height() / 2 + fm.ascent() / 2, text());
}




// #include "scrolling_label.h"
// #include <QPainter>
// #include <QPropertyAnimation>

// ScrollingLabel::ScrollingLabel(QWidget *parent) : QLabel(parent){
    
//     this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
//     auto *animation = new QPropertyAnimation(this, "geometry");
//     animation->setDuration(15000); // Duration in milliseconds
//     animation->setStartValue(QRect(parent->width(), y() + 100, parent->width(), 100));
//     animation->setEndValue(QRect((0 - parent->width()), y() + 100, parent->width(), 100));
//     animation->setLoopCount(-1); // Loop indefinitely
//     animation->start();
// }



// void ScrollingLabel::paintEvent(QPaintEvent *event) {
//     QPainter painter(this);
//     painter.drawText(rect(), Qt::AlignLeft | Qt::AlignVCenter, text());
// }
