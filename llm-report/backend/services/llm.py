
# backend/services/llm.py

import os
from openai import OpenAI
from dotenv import load_dotenv

load_dotenv()

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
MODEL_NAME = os.getenv("OPENAI_MODEL", "gpt-5-mini")

client = OpenAI(api_key=OPENAI_API_KEY) if OPENAI_API_KEY else None

def fake_response(prompt: str) -> str:
    return """
[상황 요약]
공장 내부에서 가스 누출 및 설비 과열 가능성이 감지되었습니다.

[위험 수준]
위험 상황으로 판단되며, 즉시 현장 확인이 필요합니다.

[추정 원인]
가스 배관 연결부 손상, 밸브 누설, 환기 불량 또는 설비 냉각 이상 가능성이 있습니다.

[즉시 조치]
작업자는 현장 접근을 중지하고, 보호구를 착용한 담당자만 점검해야 합니다.
필요 시 전원 차단, 환기 강화, 점화원 차단 조치를 수행하십시오.

[대피 지침]
작업자는 안전구역으로 이동하고, 가스 누출 가능성이 있는 경우 풍상측으로 대피하십시오.

[참고 근거]
LLM API 호출 실패 또는 응답 지연으로 인해 기본 안전 대응 템플릿을 반환했습니다.
"""
def call_llm(prompt: str) -> str:
    if client is None:
        print("OPENAI_API_KEY가 설정되지 않았습니다.")
        return fake_response(prompt)

    try:
        response = client.chat.completions.create(
            model=MODEL_NAME,
            messages=[
                {
                    "role": "system",
                    "content": "너는 산업안전 사고 대응 전문가다. 반드시 한국어로 간결하고 명확하게 답변한다.",
                },
                {
                    "role": "user",
                    "content": prompt,
                },
            ],
            timeout=60,
        )

        print("OpenAI 응답 성공")
        return response.choices[0].message.content

    except Exception as e:
        print("OpenAI API 오류:", e)

        return fake_response(prompt)