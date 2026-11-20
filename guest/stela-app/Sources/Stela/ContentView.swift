// The main window: one page per section, chosen in the native toolbar. The
// section control has a Liquid Glass thumb that slides between segments.
#if os(macOS)
  import SwiftUI

  @MainActor
  struct ContentView: View {
    @ObservedObject var model: AppModel

    var body: some View {
      ScrollView {
        Group {
          switch model.section {
          case .thisPC:
            ThisPCView(model: model).frame(width: 380)
          case .overview, .cpu, .updates:
            if let system = model.snapshot {
              switch model.section {
              case .cpu: CPUFeaturesView(system: system)
              case .updates: UpdatesView(model: model)
              default: OverviewView(model: model, system: system)
              }
            } else {
              ProgressView().frame(maxWidth: .infinity, minHeight: 240)
            }
          }
        }
        .frame(maxWidth: 720)
        .padding(Style.page)
        .frame(maxWidth: .infinity)
      }
      .toolbar {
        ToolbarItem(placement: .principal) {
          HStack(spacing: 0) {
            SectionSwitcher(model: model)
            Spacer(minLength: 64)
          }
          .frame(width: 384, alignment: .leading)
        }
        ToolbarItemGroup {
          Button {
            Task { await model.refresh() }
          } label: {
            Label("Refresh", systemImage: "arrow.clockwise")
          }
          .help("Refresh (⌘R)")
          .disabled(model.refreshing)
          .modifier(GlassToolbarButton())

          Button(action: model.exportReport) {
            Label("Export Report", systemImage: "square.and.arrow.up")
          }
          .help("Export Report (⇧⌘E)")
          .disabled(model.snapshot == nil)
          .modifier(GlassToolbarButton())
        }
      }
      .alert(
        "Unable to Complete Action",
        isPresented: Binding(
          get: { model.error != nil }, set: { if !$0 { model.error = nil } }
        )
      ) {
        Button("OK", role: .cancel) { model.error = nil }
      } message: {
        Text(model.error ?? "")
      }
    }
  }

  /// The standard segmented control on older macOS; a sliding glass thumb on 26+.
  private struct SectionSwitcher: View {
    @ObservedObject var model: AppModel

    var body: some View {
      if #available(macOS 26, *) {
        SlidingGlassSwitcher(selection: $model.section)
      } else {
        Picker("Section", selection: $model.section) {
          ForEach(AppSection.allCases) { Text($0.rawValue).tag($0) }
        }
        .pickerStyle(.segmented)
        .labelsHidden()
      }
    }
  }

  /// A glass thumb that lifts and follows a press-and-drag continuously, then
  /// snaps to the option it is released over. While lifted it refracts labels.
  @available(macOS 26, *)
  private struct SlidingGlassSwitcher: View {
    @Binding var selection: AppSection
    @Namespace private var namespace
    @State private var holding = false
    @State private var heldX: CGFloat?

    private let sections = AppSection.allCases
    private let sideInset: CGFloat = 6
    private var selectedIndex: Int { sections.firstIndex(of: selection) ?? 0 }

    var body: some View {
      GlassEffectContainer(spacing: 8) {
        GeometryReader { geometry in
          let width = (geometry.size.width - 2 * sideInset) / CGFloat(sections.count)
          let restingX = sideInset + CGFloat(selectedIndex) * width
          let thumbX = holding ? (heldX ?? restingX) : restingX

          ZStack(alignment: .leading) {
            HStack(spacing: 0) {
              ForEach(Array(sections.enumerated()), id: \.offset) { index, section in
                Text(section.rawValue)
                  .font(.system(size: 10, weight: .medium))
                  .frame(width: width, height: geometry.size.height)
                  .foregroundStyle(.primary)
                  .contentShape(Rectangle())
                  .accessibilityLabel(section.rawValue)
                  .accessibilityAddTraits(.isButton)
              }
            }
            .zIndex(holding ? 0 : 1)

            // At rest the glass sits behind the labels; while pressed or
            // dragged it moves in front so its refraction bends the text.
            Capsule()
              .fill(.clear)
              .glassEffect(holding ? .clear.interactive() : .clear, in: Capsule())
              .glassEffectID("thumb", in: namespace)
              .frame(width: width - 4, height: geometry.size.height - 4 + (holding ? 4 : 0))
              .offset(x: thumbX, y: 0)
              .scaleEffect(holding ? 1.06 : 1)
              .shadow(color: .black.opacity(holding ? 0.18 : 0), radius: 12, y: 6)
              // No animation while held: the thumb tracks the drag exactly.
              .animation(holding ? nil : Animation.smooth(duration: 0.28), value: selection)
              .animation(.smooth(duration: 0.2), value: holding)
              .zIndex(holding ? 1 : 0)
          }
          .contentShape(Rectangle())
          .gesture(
            DragGesture(minimumDistance: 0)
              .onChanged { value in
                let minX = sideInset
                let maxX = sideInset + CGFloat(sections.count - 1) * width
                holding = true
                heldX = min(max(value.location.x - (width - 4) / 2, minX), maxX)
              }
              .onEnded { value in
                let final = Self.clampedIndex(
                  (value.location.x - sideInset) / width, count: sections.count
                )
                withAnimation(.smooth(duration: 0.28)) {
                  selection = sections[final]
                  holding = false
                  heldX = nil
                }
              }
          )
        }
        .frame(width: 320, height: 26)
        .glassEffect(.clear, in: Capsule())
      }
      .accessibilityElement(children: .contain)
      .accessibilityLabel("Section")
    }

    private static func clampedIndex(_ value: CGFloat, count: Int) -> Int {
      guard count > 0 else { return 0 }
      return min(max(Int(value), 0), count - 1)
    }
  }

  /// Apple's documented Liquid Glass button style on macOS 26+.
  private struct GlassToolbarButton: ViewModifier {
    func body(content: Content) -> some View {
      if #available(macOS 26, *) {
        content.buttonStyle(.glass)
      } else {
        content
      }
    }
  }
#endif
