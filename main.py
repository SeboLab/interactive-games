import anki_vector
import colorsys
import rospy
import concurrent
import wizardui
import sys
from functools import partial
import threading
import time
from time import sleep
import json
from fistbump import FistbumpHandler


from PyQt5.QtWidgets import *
import data


class Timer(threading.Thread):
    def __init__(self, duration, label):
        threading.Thread.__init__(self)
        # Seconds
        self.duration = duration
        self.elapsed = 0
        self.label = label
        self.stopped = False

    def run(self):
        while self.duration and not self.stopped:
            mins, secs = divmod(self.duration, 60)

            timer = "{:02d}:{:02d}".format(mins, secs)
            self.label.setText(timer)

            sleep(1)
            self.duration -= 1
            self.elapsed += 1


dossier_info = data.hints


class MainWindow(QMainWindow, wizardui.Ui_PuzzlesWizard):
    def __init__(self, robot):
        super(self.__class__, self).__init__()
        self.setupUi(self)
        self.widget = QStackedWidget()
        self.widget.addWidget(self)
        self.widget.show()
        self.timer_thread = None
        self.robot = robot
        self.last_phrase = ""
        self.output_data = data.default_output

        phrases = data.phrases

        for button_id, phrase in phrases.items():
            getattr(self, button_id).clicked.connect(partial(self.speak, phrase))

        if self.robot is not None:
            self.lookButton.clicked.connect(
                lambda _: robot.anim.play_animation("anim_rtmotion_lookup_01")
            )

        for num in ("six", "four", "two"):
            getattr(self, f"{num}MinLeft").clicked.connect(
                partial(
                    self.speak,
                    f"Great work! You have {num} minutes left to solve this case.",
                )
            )

        self.speaking = False
        self.speakCustomButton.clicked.connect(self.say_custom_text)

        self.fistbump.clicked.connect(self.fistbump_behavior)
        self.startTimerA.clicked.connect(partial(self.start_timer, 5 * 60, "A"))
        self.startTimerB.clicked.connect(partial(self.start_timer, 10 * 60, "B"))

        self.correctSuspect.clicked.connect(self.finishA)
        self.correctHideout.clicked.connect(self.finishB)
        self.correctWeapon.clicked.connect(self.finishC)

        self.cur_dossier = "B"
        self.exportData.clicked.connect(self.export_data)

        self.repeatButton.clicked.connect(lambda: self.speak(self.last_phrase))
        self.incrementHintButton.clicked.connect(self.increment_hint)

        for dossier in ("B", "C"):
            getattr(self, f"giveHint{dossier}").clicked.connect(
                partial(self.give_hint, dossier)
            )
            getattr(self, f"incorrectProgress{dossier}").clicked.connect(
                partial(self.incorrect_progress, dossier)
            )
            getattr(self, f"correctProgress{dossier}").clicked.connect(
                partial(self.correct_progress, dossier)
            )

        self.yesButton.clicked.connect(lambda: self.yes_no_button("Yes!"))
        self.noButton.clicked.connect(lambda: self.yes_no_button("Nope!"))

    def yes_no_button(self, phrase):
        self.output_data["puzzle_progress"]["A"]["interactions"] += 1
        self.speak(phrase)

    def increment_hint(self):
        self.output_data["puzzle_progress"][self.cur_dossier]["hints"] += 1

    def export_data(self):
        for dossier in ("B", "C"):
            self.output_data["puzzle_progress"][dossier]["phases_complete"] = 0
            for phase in range(len(dossier_info[dossier]["hints"])):
                if getattr(self, f"hint{dossier}{phase + 1}Cb").isChecked():
                    self.output_data["puzzle_progress"][dossier]["phases_complete"] += 1

        participant = self.participantIdBox.toPlainText()
        self.output_data["participant"] = participant
        out = open(f"data/{participant}.json", "w+")
        json.dump(self.output_data, out)

        out.close()

    def finishA(self):
        if self.speaking:
            return
        self.output_data["puzzle_progress"]["A"]["correct"] = True
        self.output_data["puzzle_progress"]["A"]["time"] = self.timer_thread.elapsed
        self.timer_thread.stopped = True
        self.speak("I think that's the right suspect, we did it!")

    def finishB(self):
        if self.speaking:
            return
        self.output_data["puzzle_progress"]["B"]["correct"] = True
        self.output_data["puzzle_progress"]["B"]["time"] = self.timer_thread.elapsed
        self.output_data["puzzle_progress"]["C"]["time"] = self.timer_thread.duration
        self.cur_dossier = "C"
        self.speak("I think that's the right hideout, we did it!")

    def finishC(self):
        if self.speaking:
            return
        self.output_data["puzzle_progress"]["C"]["correct"] = True
        self.output_data["puzzle_progress"]["C"]["time"] = (
            self.timer_thread.elapsed - self.output_data["puzzle_progress"]["B"]["time"]
        )
        self.timer_thread.stopped = True
        self.speak("I think that's the right weapon, we did it!")

    def correct_progress(self, dossier):
        if self.speaking:
            return
        self.output_data["puzzle_progress"][dossier]["interactions"] += 1
        self.output_data["puzzle_progress"][dossier]["interactions_yes"] += 1
        self.speak("Yes!")

    def incorrect_progress(self, dossier):
        if self.speaking:
            return
        self.output_data["puzzle_progress"][dossier]["interactions"] += 1
        self.output_data["puzzle_progress"][dossier]["interactions_no"] += 1
        self.speak("Hmm, I don't think so.")
        self.give_hint(dossier)

    def start_timer(self, duration, dossier):
        if self.timer_thread is not None:
            self.timer_thread.stopped = True

        if dossier == "B":
            self.output_data["puzzle_progress"][dossier]["time"] = 600
            self.output_data["puzzle_progress"]["C"]["time"] = 0
            self.output_data["puzzle_progress"][dossier]["interactions_yes"] = 0
            self.output_data["puzzle_progress"][dossier]["interactions_no"] = 0
            self.output_data["puzzle_progress"][dossier]["hints"] = 0
        else:
            self.output_data["puzzle_progress"][dossier]["time"] = 300

        self.output_data["puzzle_progress"][dossier]["interactions"] = 0
        self.timer_thread = Timer(duration, self.timeLabel)
        self.timer_thread.start()

    def give_hint(self, dossier):
        if self.speaking:
            return
        self.output_data["puzzle_progress"][dossier]["hints"] += 1

        for phase in range(len(dossier_info[dossier]["hints"])):
            if getattr(self, f"hint{dossier}{phase + 1}Cb").isChecked():
                continue

            # Error hint
            if (
                hasattr(self, f"hint{dossier}{phase + 1}ErrorCb")
                and getattr(self, f"hint{dossier}{phase + 1}ErrorCb").isChecked()
            ):
                custom_val = ""
                if hasattr(self, f"hint{dossier}{phase + 1}ErrorVal"):
                    cur_idx = getattr(
                        self, f"hint{dossier}{phase + 1}ErrorVal"
                    ).currentIndex()
                    custom_val = dossier_info[dossier]["errorChoices"][cur_idx - 1]
                error = dossier_info[dossier]["errors"][phase]

                self.speak(error[0] + custom_val + error[1])
                return

            # Regular hint
            for i, hint in enumerate(dossier_info[dossier]["hints"][phase]):
                if getattr(self, f"hint{dossier}_{phase + 1}_{i + 1}").isChecked():
                    self.speak(hint)
                    getattr(self, f"hint{dossier}_{phase + 1}_{i + 1}").setChecked(
                        False
                    )

                    if i < len(dossier_info[dossier]["hints"][phase]):
                        getattr(self, f"hint{dossier}_{phase + 1}_{i + 2}").setChecked(
                            True
                        )

                    return

            # Repeat last hint
            self.speak(dossier_info[dossier]["hints"][phase][-1])
            return

        # No hint given, just incorrect answer
        self.output_data["puzzle_progress"][dossier]["hints"] -= 1

    def fistbump_behavior(self, mouse_event):
        if self.speaking or self.robot is None:
            return
        self.speak("Great work, give me a fistbump!")
        handler = FistbumpHandler(self.robot)
        fistbump_thread = threading.Thread(target=handler.prompt)
        fistbump_thread.start()

    def say_custom_text(self, mouse_event):
        self.speak(self.customInputBox.toPlainText())
        self.customInputBox.setPlainText("")

    def speak(self, text):
        print(f"Saying {text}")
        if self.robot is None:
            msg = QMessageBox()
            msg.setText(text)
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
        else:
            self.speaking = True
            self.robot.behavior.say_text(text, False, 0.92)
            self.speaking = False
        self.last_phrase = text


class VectorNode:
    def __init__(self, human=False, publish_rate=10, camera=False):
        self.rate = rospy.Rate(publish_rate)

        if not human:
            self.robot = anki_vector.Robot(enable_face_detection=camera)

            try:
                self.robot.connect(timeout=30)
            except anki_vector.exceptions.VectorNotFoundException:
                print("ERROR: Unable to establish a connection to Vector.")
                print(
                    "Make sure you're on the same network, and Vector is connected to the internet."
                )
                sys.exit(1)
        else:
            self.robot = None

        wizard = MainWindow(self.robot)
        wizard.show()

        if self.robot is not None:
            hue, sat, _ = colorsys.rgb_to_hsv(0, 153, 255)
            resp = self.robot.behavior.set_eye_color(hue, sat)

    def shutdown(self):
        print("Vector Robot shutting down...")
        self.robot.disconnect()


if __name__ == "__main__":

    rospy.init_node("vector_puzzles")
    app = QApplication(sys.argv)

    if len(sys.argv) > 1:
        if sys.argv[1] == "--human":
            vector = VectorNode(human=True)
        else:
            print("Invalid argument, please try again.")
            sys.exit(0)
    else:
        vector = VectorNode(human=False)

    sys.exit(app.exec_())
    rospy.spin()

    try:
        rospy.on_shutdown(vector.shutdown)
    except (concurrent.futures.CancelledError, RuntimeError):
        sys.exit(0)
