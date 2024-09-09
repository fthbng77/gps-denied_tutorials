# PrintColours.py

class PrintColours:
    CEND = '\33[0m'
    CBLINK = '\33[5m'  # Yanıp sönmeyi etkinleştiren kodu ekleyin.

    CBOLD = '\33[1m'
    # ...Diğer renk değişkenleri...

    CGREEN2 = '\33[92m'
    CYELLOW2 = '\33[93m'
    CRED2 = '\33[91m'

    @staticmethod
    def print_info(message):
        print(f"{PrintColours.CGREEN2}{message}{PrintColours.CEND}")

    @staticmethod
    def print_warning(message):
        print(f"{PrintColours.CYELLOW2}{message}{PrintColours.CEND}")

    @staticmethod
    def print_error(message):
        print(f"{PrintColours.CRED2}{message}{PrintColours.CEND}")

    @staticmethod
    def print_blinking_warning(message):
        # Sarı rengi ve yanıp sönmeyi birleştiren yeni bir yöntem.
        print(f"{PrintColours.CBLINK}{PrintColours.CYELLOW2}{message}{PrintColours.CEND}")