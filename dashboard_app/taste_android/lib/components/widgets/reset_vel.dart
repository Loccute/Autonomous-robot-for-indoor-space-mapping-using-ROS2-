import 'package:firebase_database/firebase_database.dart';

Future<void> resetAngularVel() async{
  DatabaseReference velRef = FirebaseDatabase.instance.ref();
  velRef.child("data/twist/angular_velocity").set(0.0);
}

Future<void> resetLinearVel() async{
  DatabaseReference velRef = FirebaseDatabase.instance.ref();
  velRef.child("data/twist/linear_velocity").set(0.0);
}
