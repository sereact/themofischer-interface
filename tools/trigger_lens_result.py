import requests
import json
import os
example = json.load(open("lens-executions-mock.json"))


def trigger_lens_result():
    url = "http://localhost:8080/lens_response"
    response = requests.post(url, json=example)
    print(response.text)

if __name__ == "__main__":
    trigger_lens_result()