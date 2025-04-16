import sys

from dialog_interfaces.srv import RememberInteractions
import rclpy
from rclpy.node import Node
import random


class InteractionClientAsync(Node):

    def __init__(self):
        super().__init__('interaction_client_async')
        self.cli = self.create_client(RememberInteractions, 'remember_interactions')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RememberInteractions.Request()

    def send_request(self, interaction):
        self.req.interaction = interaction
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    # interactions = ["How are you?", "What is your name?", "Where are you from?", "What people call you?"]

    # interactions = [
    #     "Hello, how can I help you?",  # Your base example

    #     # Group A: Booking a meeting (3 semantically similar but lexically different)
    #     "Remind me to call my sister in the evening.",
    #     "Set a reminder to phone my sister tonight.",
    #     "I need to remember to call my sister later today.",

    #     # Pair B: Checking the weather
    #     "Is there any rain forecast for this weekend?",
    #     "Can you tell me if it’s supposed to rain in the next few days?",

    #     # Pair C: Managing reminders
    #     "Make a note for me to buy groceries tonight.",
    #     "Remind me to stop by the store after work.",

    #     # Unrelated interactions
    #     "What does a red panda eat?",
    #     "Open the garage door.",
    #     "Tell me a scary story.",
    # ]

    interactions = [
        "Ciao, cosa posso fare per te?",  # Frase base

        # Gruppo A: promemoria (significato simile, parole diverse)
        "Ricordami di chiamare mia sorella questa sera.",
        "Imposta un promemoria per telefonare a mia sorella stasera.",
        "Devo ricordarmi di sentire mia sorella più tardi oggi.",

        # Coppia B: controllare il calendario
        "Come ti chiami?",
        "Qual è il tuo nome?",

        # Coppia C: ascoltare musica rilassante
        "Raccontami dei tuoi sentimenti.",
        "Descrivimi come ti senti.",

        # Frasi non correlate
        "Qual è la popolazione della Nuova Zelanda?",
        "Accendi il riscaldamento in camera da letto.",
        "Raccontami un indovinello.",
        "Quanto ci vuole per arrivare all’aeroporto?",
        "Qual è il senso della vita?",
        "Traduci 'grazie' in russo.",
        "Chi ha diretto il film Inception?",
        "Avvia un allenamento di 5 minuti.",
        "Quanto fa 18 per 7?"
    ]


    random.shuffle(interactions)  # Shuffle the interactions to ensure randomness

    for inter in interactions:

        interaction_client = InteractionClientAsync()
        future = interaction_client.send_request(inter)
        rclpy.spin_until_future_complete(interaction_client, future)
        response = future.result()
        if response.index == -1:
            interaction_client.get_logger().info(
                f'No previous interaction found for {inter}')
        else:
            interaction_client.get_logger().info(
                f'Most similar interaction for {inter} is at index {response.index}')

    interaction_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()