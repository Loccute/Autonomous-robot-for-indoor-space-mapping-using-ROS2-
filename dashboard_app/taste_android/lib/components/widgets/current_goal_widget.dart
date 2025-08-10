import 'package:flutter/material.dart';

class CurrentGoalWidget extends StatelessWidget {
  final int currentGoalId;
  final double estimatedTimeRemaining;
  const CurrentGoalWidget({super.key, required this.currentGoalId, required this.estimatedTimeRemaining});

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Text(
          'Current goal is: $currentGoalId',
          style: const TextStyle(
            fontSize: 13.0,
            color: Colors.black,
          ),
        ),
        Text(
          'Estimate time: ${estimatedTimeRemaining.toStringAsFixed(2)} s',
          style: const TextStyle(
            fontSize: 13.0,
            color: Colors.black,
          ),
        ),
      ],
    );
  }
}
