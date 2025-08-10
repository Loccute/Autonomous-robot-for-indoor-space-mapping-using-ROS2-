import 'package:flutter/material.dart';

class CustomColors {
  static const Color white = Color(0xFFFFFFFF);


  static const Color backgroundColors = Color(0xFFEDF3FA);
  static const Color customWhite54 = Color(0xDCFFFFFF);
  static const Color customWhite38 = Color(0x61FFFFFF);
  static const Color customWhite30 = Color(0x4DFFFFFF);
  static const Color customWhite12 = Color(0x1FFFFFFF);
  static const Color customWhite10 = Color(0x1AFFFFFF);

  // Ví dụ mô tả
  static String getDescription(Color color) {
    if (color == white) return "while";
    if (color == backgroundColors) return "Background color";
    return "None color";
  }
}
