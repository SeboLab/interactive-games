from glob import glob
import json
import csv
import sys

hints = {
    "B": {
        "hints": [
            [
                "Perhaps there's something in common in each person's transcript?",
                "Perhaps the two numbers in each transcript are important?",
            ],
            [
                "Perhaps the two numbers in each transcript can be combined?",
                "Perhaps there's a math operation hidden in each transcript?",
                "Perhaps the math operation in each transcript can be applied to the two numbers in it?",
            ],
            [
                "Perhaps you should take a look at the code sheet?",
                "Perhaps the numbers you got are all under 26?",
                "Perhaps the numbers you got can be turned into letters?",
            ],
            [
                "Perhaps the letters have a meaning?",
                "Perhaps the letters can spell a word?",
                "Perhaps the order of the letters in the word is related to something you haven't used yet?",
                "Perhaps the order of the letters is related to the names?",
                "Perhaps the names can be reordered from alphabetical to something else?",
                "Perhaps the order of the letters is related to the length of the names?",
                "Perhaps the first letter in the final word is related to the shortest name?",
                "Perhaps the second letter in the final word is related to the second-shortest name?",
            ],
        ],
        "errors": [
            (
                "Can you take a look at the numbers you got from ",
                "'s transcript again?",
            ),
            (
                "Can you take a look at the final number you got from ",
                "'s transcript again?",
            ),
            (
                "Can you take a look at the letter you got from ",
                "'s transcript again?",
            ),
            (
                "Can you take a look at the length of the names and the ordering again?",
                "",
            ),
        ],
        "errorChoices": [
            "Alexander",
            "Ben",
            "Cameron",
            "Dale",
            "Evelyn",
            "Fiona",
            "Giovanna",
        ],
    },
    "C": {
        "hints": [
            [
                "Perhaps there's something missing in each message?",
                "Perhaps we should find the missing letters in each message?",
            ],
            [
                "Perhaps the missing letters in each message have a meaning?",
                "Perhaps the missing letters in each message can spell a word?",
                "Perhaps the order of the letters is related to something you haven't used yet?",
                "Perhaps there's something in the message that's as long as the number of missing letters?",
                "Perhaps the order of the letters is related to the Message ID?",
                "Perhaps the numbers in the Message ID are related to the order of the letters?",
                "Perhaps the digit 1 in Message two three one means the corresponding letter, P,  is the first letter of the word?",
                "Perhaps the digit 1 in Message two three one means the second missing letter in the message, P, is the first letter of a word?",
                "Perhaps the digit 2 in Message two three one means the first missing letter in the message, E, should be the second letter?",
            ],
            [
                "Perhaps the words can form a phrase?",
                "Perhaps you can order the words based on the context of the messages?",
                "Perhaps the first word is from Message three five one four two?",
                "Perhaps the second word is from Message six two one four five three?",
                "Perhaps the third word is from Message two three one?",
            ],
            [
                "Perhaps the phrase you just got tells what you should do?",
                "Perhaps we can write out the third letter from each line?",
            ],
            [
                "Perhaps the letters have a meaning?",
                "Perhaps the letters can form a word?",
                "Perhaps the order of the letters corresponds with the order of the phrase Third Letter Per Line?",
            ],
        ],
        "errors": [
            (
                "Can you take a look at the missing letters you got from Message ",
                " again?",
            ),
            (
                "Can you take a look at the word you got from Message ",
                " again?",
            ),
            ("Can you take a look at where the word ", " goes again?"),
            (
                "Can you take a look at the third letter you got from Message ",
                " again?",
            ),
        ],
        "errorChoices": [
            "two three one",
            "four three two one",
            "three five one four two",
            "six two one four five three",
        ],
    },
}

phrases = {
    "oneMinLeft": "Well done! Just one minute left.",
    "timeUp": "Time is up.",
    "afterOpenA": "This is a list of suspects the Agency just sent you. I think I saw someone suspicious on the security cam. Please ask me some yes no questions so we can hone in on the suspect! Once you know the suspect, please let me know. Good luck!",
    "incorrectSuspect": "Hmm, I don't think so. Can you ask me more questions to figure it out?",
    "introA": "Hi! I'm Agent Lee, and I'm excited to work with you. Can you open Dossier A now?",
    "invalidAnswerA": "Hmm, can you tell me the person's name?",
    "invalidQuestion": "Sorry, I'm only able to answer yes or no questions based on the security camera footage.",
    "afterOpenB": "These are transcripts from town residents about where they were during the crime. They might help us figure out the suspect's hideout location. Once you figure it out, please let me know. Feel free to ask me for help or check that you're getting closer to a solution as well. Remember that the code sheet might be useful here! Good luck!",
    "introB": "Can you open Dossier B now?",
    "sampleHint": "Take a look at the words. Are there some hidden numbers?",
    "introC": "Can you open Dossier C now?",
    "afterOpenC": "This is a message the Agency sent us. It might help us figure out the suspect's weapon. Once you figure it out, please let me know. Feel free to ask me for help or check that you're getting closer to a solution as well. Good luck!",
}

default_output = {
    "participant": "",
    "puzzle_progress": {
        "A": {"time": 300, "interactions": 0, "correct": False},
        "B": {
            "time": 600,
            "hints": 0,
            "phases_complete": 0,
            "interactions": 0,
            "interactions_yes": 0,
            "interactions_no": 0,
            "correct": False,
        },
        "C": {
            "time": 0,
            "hints": 0,
            "phases_complete": 0,
            "interactions": 0,
            "interactions_yes": 0,
            "interactions_no": 0,
            "correct": False,
        },
    },
}


def json_to_csv(path):
    json_files = glob(f"{path}/*.json")

    csv_rows = list()

    for file in json_files:
        csv_data = dict()
        data = json.load(open(file))

        csv_data["participant"] = data["participant"]

        for dossier_name, dossier in data["puzzle_progress"].items():
            for data_point, value in dossier.items():
                if data_point == "hints":
                    data_point = "hints_given"
                csv_data[f"{dossier_name}.{data_point}"] = value
            if dossier_name in ("B", "C") and not ("01" in file or "02" in file):
                csv_data[f"{dossier_name}.hints_asked"] = (
                    dossier["hints"] - dossier["interactions_no"]
                )

        # Remove pilot study files
        if not ("01" in file or "02" in file):

            csv_data["total_hints"] = (
                csv_data["B.hints_given"] + csv_data["C.hints_given"]
            )
            csv_data["total_confirmations"] = (
                csv_data["B.interactions"] + csv_data["C.interactions"]
            )
            csv_data["total_phases"] = (
                csv_data["B.phases_complete"] + csv_data["C.phases_complete"]
            )
            csv_data["total_asks"] = (
                csv_data["B.hints_asked"]
                + csv_data["C.hints_asked"]
                + csv_data["total_confirmations"]
            )

        csv_rows.append(csv_data)

    header = csv_rows[-1].keys()

    with open(f"{path}/wizard_data.csv", "w", newline="") as output:
        dict_writer = csv.DictWriter(output, header)
        dict_writer.writeheader()
        dict_writer.writerows(csv_rows)


if __name__ == "__main__":
    json_to_csv(sys.argv[1])
