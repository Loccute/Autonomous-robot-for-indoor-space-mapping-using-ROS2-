import 'package:flutter/material.dart';
import 'package:numberpicker/numberpicker.dart';

class NumberPickerComponent extends StatefulWidget {
  final int maxValue;
  final Function(int newValue) onChanged;

  const NumberPickerComponent({
    super.key,
    required this.maxValue,
    required this.onChanged,
  });

  @override
  State<NumberPickerComponent> createState() => _NumberPickerComponentState();
}

class _NumberPickerComponentState extends State<NumberPickerComponent> {
  int _currentValue = 0;

  @override
  void initState() {
    super.initState();
    _currentValue = 0;
  }

  @override
  Widget build(BuildContext context) {
    return NumberPicker(
      axis: Axis.horizontal,
      value: _currentValue,
      minValue: 0,
      maxValue: widget.maxValue,
      step: 1,
      onChanged: (value) {
        setState(() {
          _currentValue = value;
        });
        widget.onChanged(value);
      },
      textStyle: const TextStyle(
        fontSize: 24,
        color: Colors.grey,
      ),
      selectedTextStyle: const TextStyle(
        fontSize: 32,
        color: Colors.blue,
        fontWeight: FontWeight.bold,
      ),
    );
  }
}
