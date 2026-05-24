# backend/services/llm.py

import os
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
MODEL_NAME = os.getenv("GEMINI_MODEL", "gemini-2.0-flash")

if GEMINI_API_KEY:
    genai.configure(api_key=GEMINI_API_KEY)
    model = genai.GenerativeModel(MODEL_NAME)
else:
    model = None


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

def call_llm(prompt):

    try:

        response = model.generate_content(
            prompt,
            request_options={
                "timeout": 15
            }
        )

        print("응답 성공")

        if not response.text:
            return fake_response(prompt)

        return response.text

    except Exception as e:

        print("Gemini API 오류:", e)

        return fake_response(prompt)
