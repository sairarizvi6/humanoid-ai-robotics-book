---

description: "Task list for Physical AI & Humanoid Robotics Textbook implementation"
---

# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-ai-robotics-textbook/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 [P] Update Docusaurus configuration in `docusaurus.config.js`
- [ ] T002 Configure sidebar generation in `sidebars.js` to list all 16 pages in order

---

## Phase 3: User Story 1 - Read Textbook Content (Priority: P1) 🎯 MVP

**Goal**: A student or educator can access and read all chapters and bonus pages of the textbook.

**Independent Test**: Accessing `intro.md`, a chapter markdown file (e.g., `docs/01-introduction.md`), and a bonus page (e.g., `docs/hardware-guide.md`) via a web browser and confirming content is displayed correctly.

### Implementation for User Story 1

- [ ] T003 [P] [US1] Create intro.md with welcome content and cover image in `docs/intro.md`
- [ ] T004 [P] [US1] Create chapter file "01-introduction.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/01-introduction.md`
- [ ] T005 [P] [US1] Create chapter file "02-embodied-intelligence.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/02-embodied-intelligence.md`
- [ ] T006 [P] [US1] Create chapter file "03-ros-2-the-robotic-nervous-system.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/03-ros-2-the-robotic-nervous-system.md`
- [ ] T007 [P] [US1] Create chapter file "04-urdf-and-robot-description.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/04-urdf-and-robot-description.md`
- [ ] T008 [P] [US1] Create chapter file "05-sensors-in-physical-ai.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/05-sensors-in-physical-ai.md`
- [ ] T009 [P] [US1] Create chapter file "06-simulation-with-gazebo.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/06-simulation-with-gazebo.md`
- [ ] T010 [P] [US1] Create chapter file "07-nvidia-isaac-sim-and-digital-twins.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/07-nvidia-isaac-sim-and-digital-twins.md`
- [ ] T011 [P] [US1] Create chapter file "08-isaac-ros-and-perception.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/08-isaac-ros-and-perception.md`
- [ ] T012 [P] [US1] Create chapter file "09-navigation-and-bipedal-locomotion.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/09-navigation-and-bipedal-locomotion.md`
- [ ] T013 [P] [US1] Create chapter file "10-vision-language-action-models.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/10-vision-language-action-models.md`
- [ ] T014 [P] [US1] Create chapter file "11-conversational-robotics-voice-to-action.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/11-conversational-robotics-voice-to-action.md`
- [ ] T015 [P] [US1] Create chapter file "12-balance-manipulation-and-whole-body-control.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/12-balance-manipulation-and-whole-body-control.md`
- [ ] T016 [P] [US1] Create chapter file "13-capstone-autonomous-humanoid-with-natural-language.md" with title, 800-1000 words, 2-3 Mermaid diagrams, ROS 2 code examples, Key Takeaways, and Practice box in `docs/13-capstone-autonomous-humanoid-with-natural-language.md`
- [ ] T017 [P] [US1] Create bonus page "Student Hardware Guide" in `docs/hardware-guide.md`
- [ ] T018 [P] [US1] Create bonus page "Recommended Humanoid Robots" in `docs/best-robots.md`

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T019 Provide instructions on how to run the Docusaurus development server and deploy the site to GitHub Pages.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **User Stories (Phase 3+)**: All depend on Setup phase completion.
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Setup (Phase 1) - No dependencies on other stories

### Within Each User Story

- Content creation (chapters, intro, bonus pages) can proceed in parallel once the basic file structure and Docusaurus configuration are in place.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 3: User Story 1
3. **STOP and VALIDATE**: Test User Story 1 independently (read content, navigate)
4. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup → Foundation ready
2. Add User Story 1 content → Test independently → Deploy/Demo (MVP!)
3. Proceed with additional content or RAG chatbot integration as separate stories.

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
