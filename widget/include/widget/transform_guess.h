#ifndef TRANSFORM_GUESS_H
#define TRANSFORM_GUESS_H

#include "configurable_widget.h"

namespace Ui {
class TransformGuess;
}

class TransformGuess : public ConfigurableWidget
{
  Q_OBJECT

public:
  explicit TransformGuess(QWidget *parent = nullptr);
  ~TransformGuess();

  void configure(const YAML::Node& node) override;
  YAML::Node save() const override;

private:
  Ui::TransformGuess *ui_;
};

#endif // TRANSFORM_GUESS_H
